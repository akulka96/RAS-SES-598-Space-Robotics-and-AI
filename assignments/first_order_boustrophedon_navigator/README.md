
## Author Information
**Name:** Atharv Rahul Kulkarni  
**Email:** akulka96@asu.edu  
**Student ID:** 1233494108

# Output
## Image
![Screenshot from 2025-01-27 23-53-29](https://github.com/user-attachments/assets/aa31cf52-b086-4d41-8213-dac77a16097a)


## 1. Introduction
This repository contains my tuned implementation of the First-Order Boustrophedon Navigator using Turtlesim in ROS2. The main goal was to create a uniform lawnmower (boustrophedon) pattern, ensuring minimal cross-track error and smooth motion. Through careful tuning of the PD controller, I achieved a precise coverage pattern that meets the assignment’s requirements for controller performance and pattern quality.

---

## 2. Methodology
I started by reviewing how lawnmower surveys typically look and confirming that each pass overlaps consistently without large gaps or excess overlap. To accomplish this, I adjusted the **spacing** to `1.2` so that each row is covered thoroughly but not excessively.

For controlling the robot, I used a **PD controller** with four main parameters:
- **Kp_linear** = 10  
- **Kd_linear** = 0.4  
- **Kp_angular** = 11  
- **Kd_angular** = 0.015  

I incrementally increased these gains from lower baseline values, observing both straight-line tracking and turning behavior at row ends. For each parameter change, I used `rqt_plot` to watch the error topics and `rqt_reconfigure` to quickly retune on the fly. Higher proportional gains helped reduce steady-state error, while small derivative terms dampened oscillations and smoothed out cornering.

---

## 3. Results
- **Final Average Cross-Track Error:** **0.041**  
- **Maximum Cross-Track Error:** **0.109**

These values indicate that the turtle consistently stayed near the desired trajectory. Sharp turns and rapid maneuvers were minimized, resulting in good coverage and smooth transitions between rows.

---

## 4. Visualization & Tools
- **Turtlesim**: Simulates the turtle robot.  
- **rqt_plot**: Plots real-time data for `/cross_track_error`, `/turtle1/pose/*`, and velocity topics.  
- **rqt_reconfigure**: (If enabled) Allows quick adjustments to controller gains without restarting the node.

I also outputted the cross-track error statistics in the terminal, which helped confirm that the navigator remained within acceptable bounds.

---

## 5. Challenges and Solutions
1. **Initial Overshoot**  
   - With higher proportional gains, the turtle initially swung wide around corners. This was fixed by gently introducing derivative terms (`Kd_linear` and `Kd_angular`) to counteract oscillations and slow down rapid heading corrections.

2. **Spacing Trade-Off**  
   - A tighter spacing made the robot turn more often, risking larger corner errors. A spacing of **1.2** proved to be a balanced choice, providing near-complete coverage with only moderate turning demands.

3. **Corner Control**  
   - Increasing `Kp_angular` from lower values helped the turtle align quickly after each turn, while `Kd_angular` ensured these turns remained smooth and didn’t produce excessive wobbling.

---

## 6. Performance Metrics & Plots
To verify the performance, I observed:

- **Cross-Track Error** over time (screenshot can be generated via `rqt_plot`).  
- **Trajectory** in the Turtlesim window, verifying it followed a neat lawnmower pattern.  
- **Velocity Commands** (`/turtle1/cmd_vel/*`) for any sudden spikes or inconsistencies.

Through these plots and observations, I concluded that my final PD settings met the coverage and error requirements stated in the assignment.

---

## 7. How to Use
1. **Clone/Fork** this repository into your `ros2_ws/src` directory.
2. **Install Dependencies** (assuming you have a ROS2 distro like Humble or Iron):
   ```bash
   sudo apt update
   sudo apt install ros-$ROS_DISTRO-turtlesim
   sudo apt install ros-$ROS_DISTRO-rqt*
   pip3 install --user numpy matplotlib
   ```
3. **Build the Package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select first_order_boustrophedon_navigator
   source install/setup.bash
   ```
4. **Run the Launch File**:
   ```bash
   ros2 launch first_order_boustrophedon_navigator boustrophedon.launch.py
   ```
5. **Observe or Adjust**:
   - View cross-track error:  
     ```bash
     ros2 topic echo /cross_track_error
     ```  
   - Real-time plots:  
     ```bash
     ros2 run rqt_plot rqt_plot
     ```
   - If desired, tweak gains in the source code or using `rqt_reconfigure` (if set up).

---

## 8. Conclusion
By systematically tuning the PD gains and adjusting the line spacing, I achieved an **average cross-track error of 0.041** and a **maximum cross-track error of 0.109**, which aligns well with the assignment’s precision targets. The result is a smooth, nearly uniform lawnmower coverage that effectively demonstrates the benefits of proper PD control in a first-order ROS2 simulation. Although I have not implemented the bonus custom message type, the core objectives of this assignment were fulfilled with satisfactory performance.
