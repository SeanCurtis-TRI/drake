#!/bin/bash

exe=./bazel-bin/examples/multibody/clutter/clutter

barrier_vals=("0" "1e-5" "1e-4" "5e-4" "1e-3")
accuracy_vals=("1e-1" "1e-2" "1e-3")

mkdir -p "clutter_data";

for b in "${barrier_vals[@]}"; do
  for ac in "${accuracy_vals[@]}"; do
    log_file="b_${b}_ac_${ac}.txt"
    out_file="stats_b_${b}_ac_${ac}.txt"
    echo "Running: d=${b}, accuracy=$ac → $out_file"
    $exe \
      --log_file=${log_file} \
      --barrier=${b} \
      --accuracy=${ac} > ${out_file} &
  done
done

wait
echo "All jobs finished."

