#!/bin/bash
rostopic echo -p /arm_1/arm_controller/velocity_command/velocities[0]/value > data.text
