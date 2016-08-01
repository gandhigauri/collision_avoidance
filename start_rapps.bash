#!/bin/bash


TURTLEBOTS="turtlebot01 turtlebot02 turtlebot03"
RAPP="collvoid_rapps/shared_pose"

for turtlebot in ${TURTLEBOTS}
do
  remappings="{remap_from: /position_share_out, remap_to: /${turtlebot}/position_share_out}, {remap_from: /position_share_in, remap_to: /${turtlebot}/position_share_in}, {remap_from: /commands_robot, remap_to: /${turtlebot}/commands_robot}, {remap_from: /transform_bf_p3, remap_to: /${turtlebot}/transform_bf_p3}, {remap_from: /transform_odom_bf, remap_to: /${turtlebot}/transform_odom_bf}, {remap_from: /transform_map_odom, remap_to: /${turtlebot}/transform_map_odom}"
  parameters=""
  rosservice call /${turtlebot}/start_rapp "${RAPP}" ["${remappings}"] ["${parameters}"] &
done

wait

exit 0


