# Message to contain execution status

uint16 NO_SIM_TO_GOAL=1
uint16 SIM_TO_GOAL=2
uint16 SIM_TO_GOAL_FORCE_REPLAN=3

uint16 NEW_ACTION_STARTED=1
uint16 ACTION_FINISHED=2
uint16 GOAL_BOOST=3

int16  executing_plan_index
ActionExecutionStatus[] executing_actions
string pddl_problem
uint16 sim_to_goal
uint16 notification_reason