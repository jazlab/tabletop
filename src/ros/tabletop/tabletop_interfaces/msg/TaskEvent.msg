# Low-rate task lifecycle event used by the passive session-health dashboard.
# Events are also recorded so timing summaries can be reproduced offline.

uint8 TASK_STARTED=1
uint8 TASK_ENDED=2
uint8 TRIAL_PRESENTED=3
uint8 TRIAL_DEPRESENTING=4
uint8 OBJECT_MISS=5
uint8 STIMULUS_REVEALED=6
uint8 MOTION_STARTED=7
uint8 GO_CUE=8

std_msgs/Header header
uint8 event_type
string session_id
int32 trial_number
string group_name
string object_id
