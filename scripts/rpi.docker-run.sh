#!/bin/bash

# Webcams work really poorly compared to the rpi cam modules.
# source $(pwd)/scripts/detect-webcams.sh

docker compose -f ./docker/rpi.compose.yml down

docker compose -f ./docker/rpi.compose.yml up