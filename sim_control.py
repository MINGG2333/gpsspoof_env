import os
import math
import numpy as np
import pyproj
import logging
import lgsvl
from simple_pid import PID

delta_steer_per_iter = 0.001

# Stanley controller parameters
k_e = 0.3
k_v = 0.01

utm_zone = 10
my_proj = pyproj.Proj("+proj=utm +zone="+str(utm_zone)+", +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")



def lonlat_to_xy(lon, lat):
    x, y = my_proj(lon, lat)
    return x, y


def xy_to_lonlat(x, y): # useful
    lon, lat = my_proj(x, y, inverse=True)
    return lon, lat


def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0*np.pi
    while angle < -np.pi:
        angle += 2.0*np.pi
    return angle


def on_left(pt, pt1, pt2):
    return (pt2[0]-pt1[0])*(pt[1]-pt1[1]) - (pt2[1]-pt1[1])*(pt[0]-pt1[0]) > 0


def distance_to_line(pt, pt1, pt2):
    slope = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
    b = pt2[1] - slope*pt2[0]
    # c = pt[1] + (pt[0]/slope) if slope != 0 else float('inf')
    dist = abs(pt[1] - slope*pt[0] - b) / math.sqrt(1+slope*slope)
    return dist


def calc_crosstrack_error(waypt, waypts):
    curr_idx = -1
    for i in range(len(waypts)):
        if abs(waypt[0] - waypts[i][0]) < 1e-5: # for timestep
            curr_idx = i
            break
    if curr_idx == -1:
        print("Waypoint matching error!")
        raise

    if curr_idx == len(waypts) - 1:
        pt1 = [waypts[curr_idx - 1][1], waypts[curr_idx - 1][2]]
        pt2 = [waypts[curr_idx][1], waypts[curr_idx][2]]
    else:
        pt1 = [waypts[curr_idx][1], waypts[curr_idx][2]]
        pt2 = [waypts[curr_idx + 1][1], waypts[curr_idx + 1][2]]
    pt = [waypt[1], waypt[2]]
    crosstrack_error = distance_to_line(pt, pt1, pt2)
    print("crosstrack_error: {}m".format(crosstrack_error))
    return crosstrack_error


class SimState:
    time = 0.
    speed = 0.
    position = None
    attitude = None


class SimControl:
    def __init__(self, frame_dir,
            sim_map="Achilles-GpsSpoofing-SanFrancisco",
            sim_veh="Achilles-GpsSpoofing-Lincoln2017MKZ",
            sim_speed=20.0):
        self.sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)
        scene_name = sim_map
        if self.sim.current_scene == scene_name:
            self.sim.reset()
        else:
            self.sim.load(scene_name, seed=100)  # fix the seed
        spawns = self.sim.get_spawn()
        state = lgsvl.AgentState()
        forward = lgsvl.utils.transform_to_forward(spawns[0])  # forward unit vector
        right = lgsvl.utils.transform_to_right(spawns[0])  # right unit vector
        lateral_shift = -4.0
        forward_shift = 0.
        state.transform.position = spawns[0].position + lateral_shift * right + forward_shift * forward
        state.transform.rotation = spawns[0].rotation
        state.velocity = sim_speed * forward
        self.ego = self.sim.add_agent(sim_veh, lgsvl.AgentType.EGO, state)
        sensors = self.ego.get_sensors()
        self.free_cam = None
        for s in sensors:
            if s.name == "3rd Person View":
                s.enabled = True
                self.free_cam = s
            else:
                s.enabled = False

        self.ego.on_collision(self.on_collision)
        # Car control knob, steering [-1, 1] +: right, -: left
        self.ctrl = lgsvl.VehicleControl()
        self.ctrl.throttle = 0.0
        self.ctrl.steering = 0.0
        self.ctrl.braking = 0.0
        self.ctrl.reverse = False
        self.ctrl.handbrake = False
        self.ego.apply_control(self.ctrl, True)

        self.sim.run(0.5)

        self.pid = PID(1.0, 0.1, 0.01, setpoint=sim_speed)
        self.pid.sample_time = 0.01
        self.pid.output_limits = (-1, 1)

        ctrl_freq = 20
        self.ctrl_period = 1.0/ctrl_freq
        self.sim_time = 0.

        self.last_steer = 0.
        self.plan_waypts = None

        self.frame_id = 0
        self.frame_dir = frame_dir
        if not os.path.exists(self.frame_dir):
            print(f"Frame folder does not exist, failed to run simulation")
            raise

    def set_plan(self, waypts):
        self.plan_waypts = waypts

    def on_collision(self, agent1, agent2, contact):
        name1 = "STATIC OBSTACLE" if agent1 is None else agent1.name
        name2 = "STATIC OBSTACLE" if agent2 is None else agent2.name
        logging.info("Collision: Autonomous vehicle collided with {}".format(name2))

    def next_yuv_frame(self):
        # freeFramePath = os.path.join(self.frame_dir, "free_frame_" + str(self.frame_id) + ".png")
        # self.free_cam.save(freeFramePath, compression=3)
        ret_frame_id = self.frame_id
        self.frame_id += 1
        return ret_frame_id

    def get_sim_state(self):
        simState = SimState()
        simState.time = self.sim_time
        simState.speed = self.ego.state.speed
        simState.position = self.sim.map_to_gps(self.ego.state.transform)
        simState.attitude = [self.ego.state.rotation.x, self.ego.state.rotation.y, self.ego.state.rotation.z]
        return simState

    def apply_control(self, throttle, steering):
        self.ctrl.throttle = throttle
        self.ctrl.steering = steering
        self.ego.apply_control(self.ctrl, True)  # sticky control
        self.sim.run(time_limit=self.ctrl_period)
        self.sim_time += self.ctrl_period

    def lateral_control(self, waypt, v, yaw):
        # Ref: https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
        yaw_path = np.arctan2(self.plan_waypts[-1][2] - self.plan_waypts[0][2], self.plan_waypts[-1][1] - self.plan_waypts[0][1])
        yaw_diff = normalize_angle(yaw_path - yaw)
        crosstrack_error = np.sqrt(np.min(np.sum((np.array(waypt[1:]) - np.array(self.plan_waypts)[:, 1:])**2, axis=1)))
        yaw_crosstrack = np.arctan2(waypt[2] - self.plan_waypts[0][2], waypt[1] - self.plan_waypts[0][1])
        yaw_path2ct = normalize_angle(yaw_path - yaw_crosstrack)
        if yaw_path2ct > 0:
            crosstrack_error = abs(crosstrack_error)
        else:
            crosstrack_error = -abs(crosstrack_error)
        yaw_diff_crosstrack = np.arctan(k_e*crosstrack_error/(k_v + v))
        steer = normalize_angle(yaw_diff + yaw_diff_crosstrack)
        # print("#####", crosstrack_error, yaw_diff, yaw_diff_crosstrack, steer)
        steer = -np.clip(steer, -1., 1.)
        steer_limited = np.clip(steer,
                self.last_steer - delta_steer_per_iter,
                self.last_steer + delta_steer_per_iter)
        self.last_steer = steer_limited
        return steer_limited

    def longitudinal_control(self, waypt, v, yaw):
        throttle = self.pid(v)
        return throttle

    def run_control(self, curr_waypt):
        if calc_crosstrack_error(curr_waypt, self.plan_waypts) > 1.0:
            raise "find calc_crosstrack_error"
        sim_state = self.get_sim_state()
        speed = sim_state.speed
        yaw = np.radians(sim_state.position[5])
        last_steering = self.ctrl.steering
        steering = self.lateral_control(curr_waypt, speed, yaw)
        throttle = self.longitudinal_control(curr_waypt, speed, yaw)
        self.apply_control(throttle, steering)
        sim_state = self.get_sim_state()
        lateral_dev = self.current_lateral_deviation(sim_state)
        return sim_state, lateral_dev

    def current_lateral_deviation(self, sim_state):
        latitude, longitude = sim_state.position[0], sim_state.position[1]
        x, y = lonlat_to_xy(longitude, latitude)
        lat_dev = np.sqrt(np.min(np.sum((np.array([x, y]) - np.array(self.plan_waypts)[:, 1:])**2, axis=1)))
        if on_left([x, y], self.plan_waypts[0][1:], self.plan_waypts[-1][1:]):
            # Define left: -, right: +
            lat_dev = -abs(lat_dev)
        else:
            lat_dev = abs(lat_dev)
        return lat_dev

