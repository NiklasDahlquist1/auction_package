

string task_type
string task_name
string task_data
int64 task_ID

string creator_name
int64 creator_ID
time created_time

float64 min_price
float64 max_price
float64 reward

bool can_be_abandoned

#bool forSpecificAgent
#string/int agent_name/agent_id



#bool can_be_reallocated_no_respons # how to handle, maybe send task (delete from server) on allocation. and allow clients to have a list of internal tasks


# add some way to say what/how many robots that are required (for use later?)



#bool useIncreasingReward
#float64 rewardIncrease # maybe make more general later?


#bool use_increasing_reward_linear


