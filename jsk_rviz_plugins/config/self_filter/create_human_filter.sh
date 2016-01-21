#!/usr/bin/env bash
human_number=$1

if [ $#  -ne 1 ]; then
    echo "Number of args is so few"
    echo "Usage ./craete_human_filter.sh human_max_id"
    exit 1
fi

for i in `seq 1 $human_number`
do
    cp human_filter.yaml human${i}_filter.yaml
    sed -e "s/{human_id}/${i}/g" -i human${i}_filter.yaml
done
