#!/bin/bash
# THE ABOVE LINE IS REQUIRED FOR THE SCRIPT TO BE EXECUTABLE AS A BASH SCRIPT - DO NOT REMOVE IT!
# Make this file executable with: chmod +x <script_name>.sh (just has to be done once)

# This is a startup script that is run when the ros docker container starts in the raspberry pi.
# That is, its just a clean way to prevent the  "command" service in the docker-compose.yml file from being too complex.