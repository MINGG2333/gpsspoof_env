# GPS Spoofing Challenge Evaluation Environment Configuration
- System requirements:
  - OS: Ubuntu 16.04, 18.04, 20.04
  - Python: 3.7+
  - GPU: a decent Nvidia GPU (e.g., GTX 1080) with Nvidia driver properly installed

## 0. Setup Python dependencies
- Install pyenv: ``./install_pyenv.sh``
- Log out and then log back in
- Install pipenv: ``./install_pipenv.sh``
- Enable the pipenv environemnt: ``pipenv shell``

## 1. Setup LGSVL simulation environment

### Simulator
- The official LGSVL simulator: ``3rd_party/lgsvlsimulator-linux64-2020.06``
- Install vulkan: ``sudo apt install libvulkan1``
  - For more details, please refer: ``https://www.svlsimulator.com/docs/archive/2020.06/getting-started/#downloading-and-starting-simulator``
- Double click to execute the simulator: ``3rd_party/lgsvlsimulator-linux64-2020.06/simulator``
- Click "Open Browser" and move to the next step
  - The browser may prompt for logging in. Since LGSVL currently has closed the account registration for the older simulator versions, please use our testing account instead: username: ``xlab4@bctf.com``, passwd: ``xlab@us``

### Setup map & vehicle resource bundles
- Map: set map name as **Achilles-GpsSpoofing-SanFrancisco**
  - Map bundle url: ``https://assets.lgsvlsimulator.com/55c89a20840315b9d20ba82cbe4db1a61240c33b/environment_SanFrancisco``
- Vehicle: Set vehicle name as **Achilles-GpsSpoofing-Lincoln2017MKZ**
  - Vehicle URL: ``https://assets.lgsvlsimulator.com/21c5f6b4ac54695a0f514d1272b6314356168cf2/vehicle_Lincoln2017MKZ``
  - Sensor conf: ``vehicle_conf.json``
- Start the "Python API" simulation in the browswer

## 2. Generate your Malicious GPS JSON file

gps.json provided by official can be used in local simulation
cp task_traj_1.json ~/Downloads/gpsspoof_env/gps.json

cp ./data/gps_pose.json ./gps.json

## 3. Run simulation
- Enable the pipenv environemnt if you haven't: ``pipenv shell``
- ``cp {your_json_file}.json ./gps.json``
- Run the simulation: ``./task_eval.sh``
- When it ends, you will see the simulation video (``sim_view.mp4``) and log (``simulation.log``) in the data folder
