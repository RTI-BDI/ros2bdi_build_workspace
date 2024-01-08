#!/bin/bash

IP=$(echo $SSH_CLIENT | awk '{print $1}')

docker run --rm -d --name=forwarder-container2000 --network=host alpine/socat TCP-LISTEN:2000,fork TCP:$IP:2000
docker run --rm -d --name=forwarder-container1234 --network=host alpine/socat TCP-LISTEN:1234,fork TCP:$IP:1234

# sudo docker-compose up -d