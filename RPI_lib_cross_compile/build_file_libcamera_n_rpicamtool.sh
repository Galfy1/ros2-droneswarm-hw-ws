#!/bin/bash
# THE ABOVE LINE IS REQUIRED FOR THE SCRIPT TO BE EXECUTABLE AS A BASH SCRIPT - DO NOT REMOVE IT!
# Make this file executable with: chmod +x <script_name>.sh (just has to be done once)
# Run this script with: ./<script_name>.sh

set -e  # Stop the script if any command fails 

# Check for required arguments
if [ -z "$1" ]; then
    echo "Error in first arg: Missing required argument <pi_username>"
    exit 1
fi

pi_username=$1

# Make sure any already existing container named "dummy" is removed before we start.
docker rm -f dummy || true  # "|| true" will ignore error if dummy does not exist (remember, we are usings "set -e" at the top of the script)
                            # this will give an "ERROR: default sources list file already exists" error if the container does not exist - we can ignore that error!


# Build with multiplatform support.
docker build . --platform linux/arm64 -t build_image

# Create a dummy container from the image
docker create --name dummy build_image

# Create a temporary directories to hold the installation files.
rm -rf temp_libcamera
mkdir -p temp_libcamera # Create the temp directory.

# # Copy the installation directory from the docker image to the host. 
# docker cp dummy/<PATH TO INSTALL FOLDER>. temp_libcamera/install 

# # And now, from host to target's home directory, but ignore COLCON_IGNORE.
# rsync -av temp_libcamera/install $pi_username@ubuntu.local:/home/$pi_username/libcamera

echo "Contents of / inside dummy container:"
docker exec dummy pwd
docker exec dummy ls -al /

docker rm -f dummy

# Clean up the temporary directories.
rm -rf temp_libcamera