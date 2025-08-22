
# ORB-SLAM3 with ZED ROS 2 Bag on Humble

This guide shows how to run **ORB-SLAM3** with a **custom ZED ROS 2 bag file** on **ROS 2 Humble**.  
It includes the required patches, environment setup, and launch configuration to make the system work reliably.

---

## 📦 Repositories Used
- **Core ORB-SLAM3 Library**: [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)  
- **ROS 2 Wrapper**: [zang09/ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2)

---

## 🔧 Step 1: Install Dependencies

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git libeigen3-dev libopencv-dev libglew-dev
````

### Install Pangolin (for visualization)

```bash
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
./scripts/install_prerequisites.sh recommended
cmake -B build
cmake --build build
sudo cmake --build build --target install
cd ..
```

---

## 📂 Step 2: Set Up the Workspace

```bash
mkdir -p ~/orb_ws/src
cd ~/orb_ws/src
```

Clone ORB-SLAM3 **outside** the workspace:

```bash
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
```

Clone the ROS 2 wrapper:

```bash
cd ~
git clone https://github.com/zang09/ORB_SLAM3_ROS2.git
```

Move the real ROS 2 package into your workspace:

```bash
mv ~/ORB_SLAM3_ROS2/ros2/orbslam3_ros2 ~/orb_ws/src/orbslam3
rm -rf ~/ORB_SLAM3_ROS2
```

---

## 🏗️ Step 3: Build Core ORB-SLAM3

```bash
cd ~/ORB_SLAM3
chmod +x build.sh
./build.sh
```

Update library path:

```bash
echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/$USER/ORB_SLAM3/Thirdparty/g2o/lib' >> ~/.bashrc
source ~/.bashrc
```

---

## 🩹 Step 4: Patch the ROS 2 Wrapper

### 4.1 Install Launch Files

Edit `CMakeLists.txt`:

```cmake
# Install launch files
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
```

### 4.2 Fix Trajectory Saving

Edit `src/stereo-inertial/stereo-inertial-node.cpp`:

```cpp
#include <unistd.h>

// Change loop
while (1)  -->  while (rclcpp::ok())

// Add at end of SyncWithImu():
std::cout << "----------------------------------------" << std::endl;
std::cout << "ROS 2 SHUTDOWN DETECTED." << std::endl;
std::cout << "Attempting to save KeyFrameTrajectory.txt..." << std::endl;
SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
std::cout << "Save command executed." << std::endl;
std::cout << "----------------------------------------" << std::endl;
```

---

## 🔨 Step 5: Build ROS 2 Workspace

```bash
cd ~/orb_ws
colcon build
```

---

## ⚙️ Step 6: Create Camera Config File

Make a file `~/orb_ws/ZED_stereo_initial.yaml` with your ZED camera intrinsics, extrinsics, and IMU parameters.

---

## 🚀 Step 7: Create Custom Launch File

Create `~/orb_ws/src/orbslam3/launch/my_zed_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orbslam3',
            executable='stereo-inertial',
            name='orbslam3_stereo_inertial',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
            arguments=[
                '/home/$USER/ORB_SLAM3/Vocabulary/ORBvoc.txt',
                '/home/$USER/orb_ws/ZED_stereo_initial.yaml',
                'false'
            ]
        )
    ])
```

Rebuild:

```bash
cd ~/orb_ws
colcon build
```

---

## ▶️ Step 8: Run the System

### Terminal 1: Start ORB-SLAM3

```bash
source ~/orb_ws/install/setup.bash
ros2 launch orbslam3 my_zed_launch.py
```

### Terminal 2: Play Bag File

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /path/to/your/bag/folder/ --clock
```

---

## 📌 Notes

* Press **Ctrl+C** in Terminal 1 to stop ORB-SLAM3.
* On shutdown, `KeyFrameTrajectory.txt` will be saved in `~/orb_ws/`.
* Ensure your `.yaml` calibration file matches your ZED camera.

---

## ✅ Done!

You should now see the ORB-SLAM3 Pangolin viewer processing your ZED bag file. 🎉

```

Do you also want me to generate a **ready-to-use YAML config template** (`ZED_stereo_initial.yaml`) with placeholders for your ZED intrinsics and IMU?
```
