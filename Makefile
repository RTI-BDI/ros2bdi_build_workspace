

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
	-v ./plansys2_ws:/root/plansys2_ws \
	-v ./tmp:/root/tmp \
	-v /Users/Shared/shared:/root/shared \
	-e "WEBOTS_SHARED_FOLDER=/Users/Shared/shared:/root/shared" \
	--rm -it \
	--name ros2bdi-build-env \
	ros2bdi-build-env \
	$(args_CMD)

EXEC = docker exec \
	-it \
	ros2bdi-build-env \
	$(if $(strip $(args_CMD)),bash -c "source /ros_entrypoint.sh; ${args_CMD}",bash)

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



.PHONY: forward-mac-start
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