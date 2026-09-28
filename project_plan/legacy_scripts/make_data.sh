#!/bin/bash

exe=./bazel-bin/examples/multibody/clutter/clutter

stiffness_vals=("1e3" "1e5" "1e7" "1e8" "1e9" "1e11" "1e13" )
margin_vals=("1e-4" "5e-4" "1e-3")
beta_vals=("16" "8" "4" "2" "1" "0.1" "0.01" "0.001")

for beta in "${beta_vals[@]}"; do
  mkdir -p "clutter_data_beta_${beta}";
done

for E in "${stiffness_vals[@]}"; do
  for d in "${margin_vals[@]}"; do
    for beta in "${beta_vals[@]}"; do
      out_file="clutter_data_beta_${beta}/E_${E}_d_${d}.txt"
      echo "Running: E=$E, d=$d, beta=$beta → $out_file"
      $exe \
        --box_stiffness=$E \
        --sphere_stiffness=$E \
        --margin=$d \
        --beta=$beta \
        --simulator_accuracy=1e-3 \
        > "$out_file" &
    done
  done
done

wait
echo "All jobs finished."

