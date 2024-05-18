# vehicle length
LENGTH = 9
# inter-vehicle distance
DISTANCE = 6
# inter-vehicle distance when leaving space for joining
JOIN_DISTANCE = DISTANCE * 2
# cruising speed
SPEED_platoon = 90 / 3.6 # 33m/s
JOINer_speed = 12
GOING_TO_POSITION = 0
OPENING_GAP = 1
COMPLETED = 2
# maneuver actors
# 这里只考虑 1 个 platoon，其中车辆编号依次为 v.0, v.1, v.2, ...
LEADER = "v.0"
# JOIN_POSITION =merge_position
N_VEHICLES = 8
JOINER = "v.%d" % N_VEHICLES
J_ID = "v.%d" % N_VEHICLES


