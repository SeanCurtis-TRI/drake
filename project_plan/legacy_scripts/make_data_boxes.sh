#!/bin/bash

exe=./bazel-bin/examples/multibody/clutter/clutter

$exe --enable_boxes --box_stiffness=1e8  --sphere_stiffness=1e7  --margin=5e-4 --objects_per_pile=10 --simulation_time=3 --sphere_resolution=0.1 > clutter_data_boxes/E_1e+07_d_5e-04.txt &
$exe --enable_boxes --box_stiffness=1e8  --sphere_stiffness=1e7  --margin=7.5e-4 --objects_per_pile=10 --simulation_time=3 --sphere_resolution=0.1 > clutter_data_boxes/E_1e+07_d_7.5e-04.txt &
$exe --enable_boxes --box_stiffness=1e8  --sphere_stiffness=1e7  --margin=1e-3 --objects_per_pile=10 --simulation_time=3 --sphere_resolution=0.1 > clutter_data_boxes/E_1e+07_d_1e-03.txt &
$exe --enable_boxes --box_stiffness=1e8  --sphere_stiffness=1e8  --margin=5e-4 --objects_per_pile=10 --simulation_time=3 --sphere_resolution=0.1 > clutter_data_boxes/E_1e+08_d_5e-04.txt &
$exe --enable_boxes --box_stiffness=1e8  --sphere_stiffness=1e8  --margin=7.5e-4 --objects_per_pile=10 --simulation_time=3 --sphere_resolution=0.1 > clutter_data_boxes/E_1e+08_d_7.5e-04.txt &
$exe --enable_boxes --box_stiffness=1e8  --sphere_stiffness=1e8  --margin=1e-3 --objects_per_pile=10 --simulation_time=3 --sphere_resolution=0.1 > clutter_data_boxes/E_1e+08_d_1e-03.txt &
$exe --enable_boxes --box_stiffness=1e9  --sphere_stiffness=1e9  --margin=5e-4 --objects_per_pile=10 --simulation_time=3 --sphere_resolution=0.1 > clutter_data_boxes/E_1e+09_d_5e-04.txt &
$exe --enable_boxes --box_stiffness=1e9  --sphere_stiffness=1e9  --margin=7.5e-4 --objects_per_pile=10 --simulation_time=3 --sphere_resolution=0.1 > clutter_data_boxes/E_1e+09_d_7.5e-04.txt &
$exe --enable_boxes --box_stiffness=1e9  --sphere_stiffness=1e9  --margin=1e-3 --objects_per_pile=10 --simulation_time=3 --sphere_resolution=0.1 > clutter_data_boxes/E_1e+09_d_1e-03.txt &
$exe --enable_boxes --box_stiffness=1e10 --sphere_stiffness=1e10 --margin=5e-4 --objects_per_pile=10 --simulation_time=3 --sphere_resolution=0.1 > clutter_data_boxes/E_1e+10_d_5e-04.txt &
$exe --enable_boxes --box_stiffness=1e10 --sphere_stiffness=1e10 --margin=7.5e-4 --objects_per_pile=10 --simulation_time=3 --sphere_resolution=0.1 > clutter_data_boxes/E_1e+10_d_7.5e-04.txt &
$exe --enable_boxes --box_stiffness=1e10 --sphere_stiffness=1e10 --margin=1e-3 --objects_per_pile=10 --simulation_time=3 --sphere_resolution=0.1 > clutter_data_boxes/E_1e+10_d_1e-03.txt &

