# Message to contain an array of plans ready to be executed, where plans[0] is supposed to be run and terminate before the execution
# of plans[1]

# Communicate what was the search baseline, i.e. the search started by assuming commitment up to a certain action,
# if empty, baseline corresponds to no commitment to any execution yet (plan search just started)

uint8 SEARCHING=0
uint8 FAILED=1
uint8 SUCCESS=2

PartialPlan[] plans
uint8 status
CommittedStatus search_baseline
int16  base_plan_index
