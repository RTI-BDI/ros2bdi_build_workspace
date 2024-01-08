#!/bin/bash

# on macOS, the forward proxy services are needed to allow the local simulation server to connect to the Webots simulator
# on Linux, these two proxy services are not needed
# on Windows, these two proxy services are not needed

export WEBOTS_HOME=/Applications/Webots.app

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

python3 $DIR/local_simulation_server.py