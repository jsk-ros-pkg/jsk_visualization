uint8 JOINT_INTERPOLATION=0
uint8 LINEAR_INTERPOLATION=1
uint8 COLLISION_AVOIDANCE=2

uint8 CLOSE_LHAND=10
uint8 CLOSE_RHAND=11
uint8 CLOSE_HANDS=12
uint8 OPEN_LHAND=13
uint8 OPEN_RHAND=14
uint8 OPEN_HANDS=15

int8 type
string arg
bool wait

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
