## Author Information
**Name:** Atharv Rahul Kulkarni  
**Email:** akulka96@asu.edu  
**Student ID:** 1233494108  

![image]![Screenshot from 2025-01-27 23-53-29](https://github.com/user-attachments/assets/bd664511-497a-4fa8-b2e6-b656c8b72e4f)


---

## 1. Introduction
This repository contains my implementation of a first-order boustrophedon (lawnmower) navigator in ROS2 using Turtlesim. The main objective was to have the turtle systematically cover a 2D area with minimal cross-track error and smooth cornering. My approach involves a proportional-derivative (PD) controller that I’ve tuned to ensure stable tracking and consistent coverage.

---

## 2. How to Use
1. **Clone or Fork** this repository into your ROS2 workspace’s `src` folder.
2. **Install Dependencies:**
   ```bash
   sudo apt update
   sudo apt install ros-$ROS_DISTRO-turtlesim
   sudo apt install ros-$ROS_DISTRO-rqt*
   pip3 install --user numpy matplotlib
   ```
3. **Build the Package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select first_order_boustrophedon_navigator
   source install/setup.bash
   ```
4. **Run the Launch File:**
   ```bash
   ros2 launch first_order_boustrophedon_navigator boustrophedon.launch.py
   ```
   This will start Turtlesim and run the boustrophedon pattern with my final PD gains.

For real-time visualization, you can also run:
```bash
ros2 run rqt_plot rqt_plot
```
and add topics like `/cross_track_error` or `/turtle1/pose/x` to observe performance.

---

## 3. Methodology
I started by reviewing standard boustrophedon (or lawnmower) patterns often used for complete coverage tasks. The navigator in this assignment plans parallel rows with a specified spacing, and the turtle transitions between rows at the edges of the space.

### Controller
- **PD Control**: I used a simple PD approach (with proportional and derivative terms) for both linear and angular velocity commands. My aim was to minimize cross-track error on the straight segments and reduce overshoot during turns.

### Tuning Steps
1. Began with lower proportional gains to avoid instability.  
2. Incrementally raised the proportional terms until the turtle tracked lines well.  
3. Added small derivative terms to smooth out oscillations on straight paths and during turns.  
4. Tweaked the spacing to ensure adequate coverage without forcing extremely tight cornering.

---

## 4. Parameter Tuning
Here are the final values that worked best in my tests:

- **Kp_linear:** 10  
- **Kd_linear:** 0.4  
- **Kp_angular:** 11  
- **Kd_angular:** 0.015  
- **Line Spacing:** 1.2  

I found these gains to provide a good balance between responsiveness and stability. The linear gains make the turtle quickly converge to each row, while the angular gains help it pivot effectively with minimal overshoot at corners.

---

## 5. Visualization & Tools
- **Turtlesim** for simulating the robot motion in a simple 2D environment.  
- **rqt_plot** to track key topics:
  - `/cross_track_error` for error monitoring
  - `/turtle1/pose/*` to watch turtle position
  - `/turtle1/cmd_vel/*` for velocity commands  
- **rqt_reconfigure** can be used to experiment with different gains live (if you have the relevant package installed).

---

## 6. Challenges and Solutions
- **Initial Overshoot**: With high proportional gains, the turtle would wildly overshoot the desired path. Lowering `Kp` and introducing a small `Kd` helped dampen these oscillations.
- **Corner Oscillations**: Sharp turns at the row ends caused noticeable error spikes. Increasing `Kp_angular` and adding a small `Kd_angular` term made cornering tighter and smoother.
- **Spacing vs. Coverage**: If spacing was too large, the turtle made abrupt U-turns. Adjusting spacing to 1.2 minimized extreme steering while still covering the area effectively.

---

## 7. Performance Metrics & Plots
- **Final Average Cross-Track Error**: **0.041**  
- **Maximum Cross-Track Error**: **0.109**  

These values reflect how well the turtle adhered to each row throughout the run.  
You can see an example of the error trend or the turtle’s path in the plot above (`my_output.png`), which shows relatively tight tracking and smoother corner transitions compared to earlier runs.

If you’d like to generate more plots yourself, simply pipe the data into `rqt_plot` or a custom Python script that subscribes to `/cross_track_error`.

---

## 8. Conclusion
This project demonstrates that careful PD tuning can significantly improve coverage path tracking for a first-order robot model in ROS2. With the final gains listed above, the turtle follows a consistent lawnmower path, maintains an average cross-track error of around 0.041 (well below the target threshold), and keeps corners neat without substantial overshoot.  

There’s always room for further refinement, but I’m satisfied with these results given the goals and constraints of this assignment. If you have any questions or suggestions, feel free to reach out via my email above.

---
```

