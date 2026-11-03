# Collision data message containing obstacle and vehicle collision counts and states

# Total count of obstacle collision transitions (false -> true)
uint64 obstacle_collision_count

# Total count of vehicle collision transitions (false -> true)
uint64 vehicle_collision_count

# True when the vehicle collider overlaps any OBSTACLE body
bool in_collision_with_obstacle

# True when the vehicle collider overlaps any other VEHICLE body
bool in_collision_with_vehicle
