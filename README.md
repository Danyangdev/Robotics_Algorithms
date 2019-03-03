# Robotics_Algorithms
# Implemented some simple algorithms
# 1. A* Algorithm
#   Get current odometry data from subscriber of base_pse_ground_truth 
#   Judge if the it is doing rotation
#   If so, judge if current angle able to rotate, if so, get rotation data, and publish to the cmd_vel
#   Then get Odometry, judge if the current angle able to rotate, if not then change global veriable rot to false
#   Then do move, first judge if the distence is long engough to do movement,
#   If so,do movement
#   Then get Odometry , judge if the current distence able to move
#   If not, change rot to True, because next time it will rotate, and change the current index to the next if the next is not the end
#   then do this operation again until the robot arrives to the end point.
# 2. Kalman Filter
