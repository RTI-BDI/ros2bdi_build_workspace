

.DEFAULT_GOAL := help



args = $(filter-out $@,$(MAKECMDGOALS))
args_CMD = $(if $(strip $(args)),$(args),$(CMD))

BUILD = docker build \
	--platform=linux/amd64 \
	--rm  \
	--tag ros2bdi-build-env \
	-f DockerfileBuildEnv \
	.

RUN = docker run \
	--shm-size=4g \
	--platform=linux/amd64 \
	-v ./ros2bdi_ws:/root/ros2bdi_ws \
	-v ./tmp:/root/tmp \
	-v /Users/Shared/shared:/root/shared \
	-e "WEBOTS_SHARED_FOLDER=/Users/Shared/shared:/root/shared" \
	-e "USERNAME=root" \
	-v /tmp/.X11-unix/:/tmp/.X11-unix/ \
	-v ${HOME}/.Xauthority:/root/.Xauthority:rw \
	-e "DISPLAY=host.docker.internal:0" \
	--rm -it \
	--name ros2bdi-build-env \
	ros2bdi-build-env \
	$(args_CMD)

EXEC = docker exec \
	-it \
	ros2bdi-build-env \
	bash --init-file /ros_entrypoint.sh -i $(if $(strip $(args_CMD)),-c "$(args_CMD)",)

IF_CONTAINER_RUNS=$(shell docker container inspect -f '{{.State.Running}}' ros2bdi-build-env 2>/dev/null)

ifeq (${IF_CONTAINER_RUNS},true)
LOGIN = ${EXEC}
else
LOGIN = ${RUN}
endif



FORWARD2000=$(shell docker container inspect -f '{{.State.Status}}' forwarder-container2000 2>/dev/null)
FORWARD1234=$(shell docker container inspect -f '{{.State.Status}}' forwarder-container1234 2>/dev/null)
ifeq ($(FORWARD2000),)
	FORWARD2000 = "not_found"
endif
ifeq ($(FORWARD1234),)
	FORWARD1234 = "not_found"
endif






.PHONY: help
help:
	@echo "Webots executes natively on host machine, be sure to run webots server."

	@printf "On mac, docker runs on VM, be sure to run forwarding services from VM to host network. ";
	@if [ "${FORWARD2000}" == "running" ] && [ "${FORWARD1234}" == "running" ]; \
	then \
		printf "Forwarding services are RUNNING"; \
	else \
		printf "Forwarding services NOT running"; \
	fi; echo

	@echo
	@cat $(MAKEFILE_LIST) | grep -E '^[a-zA-Z_-]+:.*?## .*$$' | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'
	@echo


.PHONY: docker-login
docker-login: ## Login to the container. If the container is already running, login into existing one.
	xhost +localhost
	@echo ${LOGIN}
	@echo
	@${LOGIN}

.PHONY: docker-build
docker-build: ## Build the docker ros2bdi-build-env image.
	${BUILD}
	@echo
	@echo "Build finished. Docker image name: \"ros2bdi-build-env\"."

.PHONY: run-world
run-world: ## Login and run world.
	@make docker-login CMD="ros2 launch webots_ros2_simulations blocks_world.launch.py;"

.PHONY: run-carrier_a
run-carrier_a: ## Login and run carrier_a.
	@make docker-login ros2 launch ros2_bdi_on_webots carrier_a.launch.py

.PHONY: run-carrier_b
run-carrier_b: ## Login and run carrier_b.
	@make docker-login ros2 launch ros2_bdi_on_webots carrier_b.launch.py

.PHONY: run-carrier_c
run-carrier_c: ## Login and run carrier_c.
	@make docker-login ros2 launch ros2_bdi_on_webots carrier_c.launch.py

.PHONY: run-gripper_a
run-gripper_a: ## Login and run gripper_a.
	@make docker-login ros2 launch ros2_bdi_on_webots gripper_a.launch.py

.PHONY: run-all
run-all: ## Login and run all.
	@make docker-login CMD="bash -c \"\
		ros2 launch webots_ros2_simulations blocks_world.launch.py & sleep 5; \
		ros2 launch ros2_bdi_on_webots gripper_a.launch.py & sleep 10; \
		ros2 launch ros2_bdi_on_webots carrier_a.launch.py & sleep 10; \
		ros2 launch ros2_bdi_on_webots carrier_b.launch.py & sleep 10; \
		ros2 launch ros2_bdi_on_webots carrier_c.launch.py\" "



.PHONY: run-plastic
run-plastic: ## Login and run plastic.
	@make docker-login ros2 launch ros2_bdi_on_litter_world plastic_agent.launch.py

.PHONY: run-paper
run-paper: ## Login and run paper.
	@make docker-login ros2 launch ros2_bdi_on_litter_world paper_agent.launch.py

.PHONY: run-litter
run-litter: ## Login and run litter-world.
	@make docker-login CMD="ros2 run litter_world litter_world_ros2_controller --ros-args -p init_world:=./src/ROS2-BDI/simulations/litter_world/init/c_7x7.json -p upd_interval:=1600 -p show_agent_view:=plastic_agent -p world_size_px:=400"



.PHONY: forward-stop
forward-stop:
	docker stop forwarder-container2000
	docker stop forwarder-container1234



.PHONY: forward-ssh
forward-ssh: ## After ssh remote host, run this to start forwarding requests toward local host.
	@IP=$$(echo $(SSH_CLIENT) | awk '{print $$1}'); export IP=$$IP; echo $$IP
	@if [ "${FORWARD2000}" != "running" ]; then \
		docker run \
		--rm -d \
		--name=forwarder-container2000 \
		--network=host \
		alpine/socat \
		TCP-LISTEN:2000,fork \
		TCP:${IP}:2000; \
	fi

	@if [ "${FORWARD1234}" != "running" ]; then \
		docker run \
		--rm -d \
		--name=forwarder-container1234 \
		--network=host \
		alpine/socat \
		TCP-LISTEN:1234,fork \
		TCP:${IP}:1234; \
	fi



.PHONY: forward-mac
forward-mac:
	@if [ "${FORWARD2000}" != "running" ]; then \
		docker run \
		--rm -d \
		--name=forwarder-container2000 \
		--network=host \
		alpine/socat \
		TCP-LISTEN:2000,fork \
		TCP:host.docker.internal:2000; \
	fi

	@if [ "${FORWARD1234}" != "running" ]; then \
		docker run \
		--rm -d \
		--name=forwarder-container1234 \
		--network=host \
		alpine/socat \
		TCP-LISTEN:1234,fork \
		TCP:host.docker.internal:1234; \
	fi



.PHONY: webots-mac
webots-mac: forward-mac ## Run local simulation server for Webots on mac.
	export WEBOTS_HOME=/Applications/Webots.app
	python3 $(CURDIR)/scripts/local_simulation_server.py
	docker stop forwarder-container2000
	docker stop forwarder-container1234



.PHONY: webots-linux
webots-linux: ## Run local simulation server for Webots on linux.
	export WEBOTS_HOME=/usr/local/webots"
	python3 $(CURDIR)/scripts/local_simulation_server.py



# catch all target (%) which does nothing to silently ignore all the other "goals" when used as args.
%:
	@true 