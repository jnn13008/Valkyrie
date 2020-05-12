#!/bin/bash
init=100
div=100
dec=10
for (( i = $init; i > 0; i=$i - $dec )); do
    rosparam set valkyrie/eff_step  $(echo "$i/$div" | bc -l)
    echo "eff_step = $(rosparam get valkyrie/eff_step)"
    echo "jump_th = $(rosparam get valkyrie/jump_th)"
    rosservice  call valkyrie/setstate  1 right
done
rosservice  call valkyrie/setstate  0 home
