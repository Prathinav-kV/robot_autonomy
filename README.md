# 🏎️ TurtleBot3 Obstacle Avoidance & Teleoperation (ROS 2 Humble)

This project demonstrates **teleoperation and autonomous obstacle avoidance** for **TurtleBot3** in **ROS 2 Humble**.

---

## 📥 1️⃣ Cloning the Repository

First, **clone the repository** from GitHub:

```bash
git clone https://github.com/Prathinav-kV/robot_autonomy.git
cd robot_autonomy
```

---

## 🔨 2️⃣ Building the Package with Colcon

After cloning, **build the package** using `colcon`:

```bash
colcon build --packages-select robot_autonomy
source install/setup.bash
```

---

## 🌍 3️⃣ Launching TurtleBot3 in Gazebo

Before running any control nodes, **start the TurtleBot3 simulation**:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## 🎮 4️⃣ Running Teleoperation (Part 1)

To manually control the TurtleBot3 using a keyboard:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Controls:**

- `W` → Move Forward
- `S` → Move Backward
- `A` → Turn Left
- `D` → Turn Right
- `X` → Stop

✅ This step **demonstrates Part 1 (teleoperation).**

---

## 🚀 5️⃣ Running Autonomous Obstacle Avoidance (Part 2)

Now, run the **obstacle avoidance** node with different parameter values.

### **Run with ****`linear_velocity=0.3`**

```bash
ros2 run robot_autonomy obstacle_avoidance --ros-args -p linear_velocity:=0.3
```

### **Run with ****`linear_velocity=0.5`**

```bash
ros2 run robot_autonomy obstacle_avoidance --ros-args -p linear_velocity:=0.5
```

✅ This step **demonstrates Part 2 (autonomous navigation).**

---

## 🐳 6️⃣ Running `obstacle_avoidance` Inside Docker

If you want to run everything inside Docker:

### **1️⃣ Start TurtleBot3 in Gazebo (Inside Docker)**

```bash
docker run -it --rm --net=host \
    --env DISPLAY=$DISPLAY \
    --env TURTLEBOT3_MODEL=waffle \
    --env ROS_DOMAIN_ID=0 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    robot_autonomy ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### **2️⃣ Run ****`obstacle_avoidance`**** in Docker**

```bash
docker run -it --rm --net=host \
    --env ROS_DOMAIN_ID=0 \
    robot_autonomy ros2 run robot_autonomy obstacle_avoidance --ros-args -p linear_velocity:=0.2
```

---

## 🎯 Summary of Commands

| **Step**                                         | **Command**                                                                                                                   |
| ------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------- |
| **Clone Repo**                                   | `git clone https://github.com/Prathinav-kV/robot_autonomy.git`                                                                |
| **Build Package**                                | `colcon build --packages-select robot_autonomy`                                                                               |
| **Launch Gazebo**                                | `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`                                                                    |
| **Run Teleop**                                   | `ros2 run turtlebot3_teleop teleop_keyboard`                                                                                  |
| **Run Obstacle Avoidance (****`0.3`**** speed)** | `ros2 run robot_autonomy obstacle_avoidance --ros-args -p linear_velocity:=0.3`                                               |
| **Run Obstacle Avoidance (****`0.5`**** speed)** | `ros2 run robot_autonomy obstacle_avoidance --ros-args -p linear_velocity:=0.5`                                               |
| **Run Gazebo in Docker**                         | `docker run -it --rm --net=host robot_autonomy ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`                      |
| **Run Obstacle Avoidance in Docker**             | `docker run -it --rm --net=host robot_autonomy ros2 run robot_autonomy obstacle_avoidance --ros-args -p linear_velocity:=0.2` |

🚀 **Now you’re ready to test teleoperation & obstacle avoidance for TurtleBot3 in ROS 2 Humble!**

