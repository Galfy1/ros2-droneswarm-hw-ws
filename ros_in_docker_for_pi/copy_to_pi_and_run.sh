#!/bin/bash
# THE ABOVE LINE IS REQUIRED FOR THE SCRIPT TO BE EXECUTABLE AS A BASH SCRIPT - DO NOT REMOVE IT!
# Make this file executable with: chmod +x <script_name>.sh (just has to be done once)
# Run this script with: ./<script_name>.sh


set -e  # Stop the script if any command fails


pi_hostname="raspberrypi.local"
target_dir=~/ros2_droneswarm


# We could build the docker on the pi

# TODO: byg på pc og copy image over
rsync -av --mkpath Dockerfile docker-compose.yml startup_script.sh $pi_hostname:$target_dir



# restart dockeren.