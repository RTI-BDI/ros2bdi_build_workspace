#!/bin/bash

# on macOS, the following two proxy services are needed to allow the local simulation server to connect to the Webots simulator
# on Linux, these two proxy services are not needed
# on Windows, these two proxy services are not needed

docker run --rm -d --name=forwarder-container2000 --network=host alpine/socat TCP-LISTEN:2000,fork TCP:host.docker.internal:2000
docker run --rm -d --name=forwarder-container1234 --network=host alpine/socat TCP-LISTEN:1234,fork TCP:host.docker.internal:1234

# On docker-compose these two proxy services could be started as:
# 
#   # docker run --rm -d --name=forwarder-container2000 --network=host alpine/socat TCP-LISTEN:2000,fork TCP:host.docker.internal:2000
#   forwarder-2000:
#     image: alpine/socat
#     command: TCP-LISTEN:2000,fork TCP:host.docker.internal:2000
#     container_name: forwarder-container2000
#     network_mode: host
# 
#   # docker run --rm -d --name=forwarder-container1234 --network=host alpine/socat TCP-LISTEN:1234,fork TCP:host.docker.internal:1234
#   forwarder-1234:
#     image: alpine/socat
#     command: TCP-LISTEN:1234,fork TCP:host.docker.internal:1234
#     container_name: forwarder-container1234
#     network_mode: host

export WEBOTS_HOME=/Applications/Webots.app

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

python3 $DIR/local_simulation_server.py