#!/bin/bash
# THE ABOVE LINE IS REQUIRED FOR THE SCRIPT TO BE EXECUTABLE AS A BASH SCRIPT - DO NOT REMOVE IT!
# Make this file executable with: chmod +x <script_name>.sh (just has to be done once)
# Run this script with: ./<script_name>.sh

# THIS SCRIPT COPIES THE DOCKERFILE AND DOCKER-COMPOSE.YML TO THE PI

set -e  # Stop the script if any command fails

pi_username="devboard"
pi_hostname="raspberrypi.local"
target_dir=~/ros2_droneswarm

rsync -av --mkpath Dockerfile docker-compose.yml $pi_username@$pi_hostname:$target_dir