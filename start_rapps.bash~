#!/bin/bash


TURTLEBOTS="turtlebot01 turtlebot02 turtlebot03"
RAPP="collvoid_rapps/shared_pose"

for turtlebot in ${TURTLEBOTS}
do
  remappings="{remap_from: /position_share_out, remap_to: /${turtlebot}/position_share_out}, {remap_from: /position_share_in, remap_to: /${turtlebot}/position_share_in}, {remap_from: /commands_robot, remap_to: /${turtlebot}/commands_robot}"
  parameters=""
  rosservice call /${turtlebot}/start_rapp "${RAPP}" ["${remappings}"] ["${parameters}"] &
done

wait

exit 0


