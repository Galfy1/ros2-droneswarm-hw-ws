#!/bin/bash
# THE ABOVE LINE IS REQUIRED FOR THE SCRIPT TO BE EXECUTABLE AS A BASH SCRIPT - DO NOT REMOVE IT!
# Make this file executable with: chmod +x <script_name>.sh (just has to be done once)
# Run this script with: ./<script_name>.sh

set -e

install_dependencies="no"

pi_hostname="raspberrypi.local"
ssh $pi_hostname << EOF
    echo "sushi1"
    echo "sushi2"
    echo "sushi3"
    echo "sushi4"
    echo "sushi5"
    echo "sushi6"
    if [ "$install_dependencies" = "yes" ]; then
        echo "Installing dependencies..."
        # Add commands to install dependencies here
    fi
EOF