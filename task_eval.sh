#!/bin/bash

input_pose=$(realpath ./gps.json)
logdir=$(realpath ./data)

mkdir -p ${logdir}

python task_eval.py --conf task_conf.yml --task 0 --input_pose ${input_pose} --logdir ${logdir}

ffmpeg -r 20 -i ${logdir}/frames/free_frame_%d.png -vcodec libx264 -pix_fmt yuv420p ${logdir}/sim_view.mp4 -y
