#!/usr/bin/env bash

if [ ! $# -eq 1 ]; then
    echo "Usage:"
    echo " ./create_human_xacros.sh NUM"
    echo ""
    echo "NUM should be decided with openni_tracker bone_name ex) torso_1 -> 1"
    exit
fi

human_number=$1

for i in `seq 1 $human_number`
do
    cp human.urdf.xacro human${i}.urdf.xacro
    sed -e "s/{human_id}/${i}/g" -i human${i}.urdf.xacro
done
