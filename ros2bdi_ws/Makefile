

.DEFAULT_GOAL := help



.PHONY: help
help:
	@cat $(MAKEFILE_LIST) | grep -E '^[a-zA-Z_-]+:.*?## .*$$' | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'



.PHONY: run-world
run-world: ## Run world.
	ros2 launch webots_ros2_simulations blocks_world.launch.py

.PHONY: run-carrier_a
run-carrier_a: ## Run carrier_a.
	ros2 launch ros2_bdi_on_webots carrier_a.launch.py

.PHONY: run-carrier_b
run-carrier_b: ## Run carrier_b.
	ros2 launch ros2_bdi_on_webots carrier_b.launch.py

.PHONY: run-carrier_c
run-carrier_c: ## Run carrier_c.
	ros2 launch ros2_bdi_on_webots carrier_c.launch.py

.PHONY: run-gripper_a
run-gripper_a: ## Run gripper_a.
	ros2 launch ros2_bdi_on_webots gripper_a.launch.py

.PHONY: run-all
run-all: ## Run all.
	bash -c \"\
	ros2 launch webots_ros2_simulations blocks_world.launch.py & sleep 5; \
	ros2 launch ros2_bdi_on_webots gripper_a.launch.py & sleep 10; \
	ros2 launch ros2_bdi_on_webots carrier_a.launch.py & sleep 10; \
	ros2 launch ros2_bdi_on_webots carrier_b.launch.py & sleep 10; \
	ros2 launch ros2_bdi_on_webots carrier_c.launch.py\"



.PHONY: run-plastic
run-plastic: ## Run plastic.
	ros2 launch ros2_bdi_on_litter_world plastic_agent.launch.py

.PHONY: run-paper
run-paper: ## Run paper.
	ros2 launch ros2_bdi_on_litter_world paper_agent.launch.py

.PHONY: run-litter
run-litter: ## Run litter.
	ros2 run litter_world litter_world_ros2_controller --ros-args -p init_world:=./src/ROS2-BDI/simulations/litter_world/init/c_7x7.json -p upd_interval:=1600 -p show_agent_view:=plastic_agent -p world_size_px:=400
