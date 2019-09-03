#!/bin/bash

# set gazebo physics step size so that it uses less CPU resources

sleep 5
gz physics --step-size 0.0015 --update-rate 667 --iters 30
