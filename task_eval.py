#!/usr/bin/env python

import os
import sys
import json
import yaml
import time
import logging
import cv2
import argparse
import matplotlib.pyplot as plt
import folium

from sim_control import SimControl, lonlat_to_xy, xy_to_lonlat

SIM_FRAMES = 200

parser = argparse.ArgumentParser(argument_default=argparse.SUPPRESS)
parser.add_argument('--conf', type=str, help='Config file')
parser.add_argument('--task', type=int, help='Task id, e.g., 0')
parser.add_argument('--input_pose', type=str, help='Spoofed GPS input file submitted by player')
parser.add_argument('--logdir', type=str, help='Folder to store evaluation results')


def get_task_conf(conf_file, task_id):
    conf = None
    with open(conf_file, 'r') as f:
        all_conf = yaml.load(f, Loader=yaml.FullLoader)
        if not task_id in all_conf['task_id']:
            failed("task id is invalid: " + str(task_id))
            return
        conf = all_conf[task_id]
    return conf


def parse_pose(pose_file, plot=False):
    with open(pose_file, 'r') as f:
        waypts_lla = json.load(f)
    if plot:
        m = folium.Map(location=[waypts_lla[0]['latitude'], waypts_lla[0]['longitude']],
                max_zoom=25, zoom_start=18)
        for pt in waypts_lla:
            folium.CircleMarker([pt['latitude'], pt['longitude']],
                    radius=2, fill=True, color='black', popup=f"{pt['timestamp']}").add_to(m)
        m.save("result.html")

    pts = []
    for pt in waypts_lla:
        x, y = lonlat_to_xy(pt['longitude'], pt['latitude'])
        pts.append([pt['timestamp'], x, y])
    return pts


def task_eval(logdir, benign_pose_file, input_pose_file, **kwargs):
    waypts_plan = parse_pose(benign_pose_file, plot=True)
    waypts_input = parse_pose(input_pose_file, plot=True)
    if len(waypts_plan) != len(waypts_input):
        print("Number of GPS inputs is incorrect:", len(waypts_input), "vs", len(waypts_plan))
        raise
    frame_dir = os.path.join(logdir, 'frames')
    if not os.path.exists(frame_dir):
        os.mkdir(frame_dir)

    sim_ctrl = SimControl(frame_dir)
    sim_ctrl.set_plan(waypts_plan)
    time.sleep(1)
    sim_state = sim_ctrl.get_sim_state()
    poses = []
    poses.append({
        "timestamp": round(sim_state.time, 2),
        "latitude": sim_state.position[0],
        "longitude": sim_state.position[1],
        "altitude": sim_state.position[4]})
    waypt_idx = 0
    max_lat_dev = 0.
    min_lat_dev = float('inf')
    frame_count = 0
    while True:
        frame_id = sim_ctrl.next_yuv_frame()

        # modify gps in xy
        if waypt_idx < 10:
            waypts_input[waypt_idx][-1] += 0.394 / 10 * waypt_idx
        elif waypt_idx < 20:
            waypts_input[waypt_idx][-1] += (0.394 - 0.394 / 10 * (waypt_idx - 10))
        elif waypt_idx < 25:
            waypts_input[waypt_idx][-1] += -0.0
        elif waypt_idx < 40:
            waypts_input[waypt_idx][-1] += 0.394
        else:
            waypts_input[waypt_idx][-1] += 0.394

        # simulation tick
        curr_waypt = waypts_input[waypt_idx] # 202 steps
        logging.info("Time {:.2f}, max left deviation {:.2f}, max right deviation {:.2f}".format(sim_ctrl.sim_time, abs(min(min_lat_dev, 0.0)), abs(max_lat_dev)))
        sim_state, lat_dev = sim_ctrl.run_control(curr_waypt)
        max_lat_dev = max(max_lat_dev, lat_dev)
        min_lat_dev = min(min_lat_dev, lat_dev)
        poses.append({
            "timestamp": round(sim_state.time, 2),
            "latitude": sim_state.position[0],
            "longitude": sim_state.position[1],
            "altitude": sim_state.position[4]})

        waypt_idx += 1
        frame_count += 1
        if frame_id >= SIM_FRAMES:
            break

    # Save pose
    pose_file = os.path.join(logdir, 'adc_pose.json')
    with open(pose_file, 'w') as f:
        json.dump(poses, f, indent=2)

    # gps pose
    gps_pose0 = [[stp]+list(xy_to_lonlat(x, y))+[10.117] for stp, x, y in waypts_input]
    gps_pose = [{
            "timestamp": round(stp, 2),
            "latitude": lat,
            "longitude": lon,
            "altitude": altitude} for stp, lon, lat, altitude in gps_pose0]
    with open(os.path.join(logdir, 'gps_pose.json'), 'w') as f:
        json.dump(gps_pose, f, indent=2)


def main():
    args = parser.parse_args()
    challenge_dir = os.path.dirname(os.path.abspath(args.conf))
    conf = get_task_conf(args.conf, args.task)
    benign_pose_file = os.path.join(challenge_dir, conf['args_eval']['plan_traj'])
    input_pose_file = args.input_pose
    logdir = os.path.abspath(args.logdir)

    logging.basicConfig(filename=os.path.join(logdir, "simulation.log"),
            level=logging.INFO, format="[%(asctime)s] %(message)s")

    task_eval(logdir, benign_pose_file, input_pose_file, **conf['args_eval'])


if __name__ == "__main__":
    main()
