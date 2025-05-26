# Planar 2-DOF Manipulator Control with Peter Corke's Robotics Toolbox

This project simulates and controls a planar 2-DOF robotic arm using MATLAB and the Peter Corke Robotics Toolbox. It includes trajectory generation, closed-loop PD control with gravity compensation, and adaptive Lyapunov-based control.

## ✅ Features
  - Robot modeling using DH parameters (SerialLink)

  - Smooth joint trajectory generation (jtraj)

  - +PD and Lyapunov-based controllers

  - Disturbance estimation (d̂) and animation of motion

## 📁 Files
  define_planar_robot.m — Robot model
  
  generate_trajectory.m — Joint trajectory
  
  simulate_PD_tracking.m — Closed-loop PD simulation
  
  Lyapunov_Integral_Controller_With_Estimation.m — Adaptive controller
