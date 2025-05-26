# Planar 2-DOF Manipulator Control with Peter Corke's Robotics Toolbox

This project implements simulation, control, and animation of a 2-DOF planar elbow manipulator using MATLAB and the Peter Corke Robotics Toolbox. It includes:

- Robot modeling using DH parameters and SerialLink
- Trajectory generation using `jtraj`
- Closed-loop control with PD + gravity compensation
- Inverse dynamics with `rne`, `inertia`, `coriolis`, and `gravload`
- Simulation and animation of joint-space behavior

---

## 📁 Project Structure

```bash
📦planar-2dof-manipulator-control/
├── define_planar_robot.m            # Builds the robot using SerialLink
├── generate_trajectory.m            # Creates joint-space trajectory
├── simulate_PD_tracking.m           # Runs closed-loop simulation
├── Lyapunov_Integral_Controller_With_Estimation.m  # Advanced controller (optional)
├── README.md
