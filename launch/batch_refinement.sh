#!/bin/bash

config_file=`rospack find oa_licalib`"/config/structor.yaml"
echo "$config"

date_str="$(date +"%Y-%m-%d_%H:%M:%S")"
output_dir="/home/$USER/Documents/OACalib/${date_str}"


window_size="10" # sekunder [s] heltal
shift_step="5" #heltal
total_size="100" #längd på rosbag





#Algorithm
counter="0"
for ((i=5; i+window_size <= total_size; i+=shift_step))
do
  counter=$((counter+1))
  start=$i
  end=$((i+window_size))
  echo "Window: [$start, $end)"
  
  output_dir_counter="${output_dir}/${counter}/"
  echo "$output_path"
  mkdir -p ${output_dir_counter} 
  output_path="${output_dir_counter}"
  pars="--config_file ${config_file} --start_time ${start} --end_time ${end} --output_path ${output_path} " 
  echo "  rosrun oa_licalib li_calib_node  $pars"
#  rosrun oa_licalib li_calib_node ${pars} &> /dev/null
done




