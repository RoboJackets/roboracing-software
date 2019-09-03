#!/bin/bash

# set gazebo physics step size so that it uses less CPU resources

sleep 5
gz physics --step-size 0.0025 --update-rate 400 --iters 30
