#!/bin/bash
# Emergency script to kill the control loop
echo "Finding and killing run_g1_control_loop processes..."
pkill -9 -f "run_g1_control_loop.py"
echo "Done. All processes killed."
