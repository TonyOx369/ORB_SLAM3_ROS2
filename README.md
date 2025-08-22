
# ORB-SLAM3 with ZED ROS 2 Bag on Humble

This guide shows how to run **ORB-SLAM3** with a **custom ZED ROS 2 bag file** on **ROS 2 Humble**.  
It includes the required patches, environment setup, and launch configuration to make the system work reliably.

---

## 📦 Repositories Used
- **Core ORB-SLAM3 Library**: [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
  
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

### Install OpenCV 4.2 from Source

ORB-SLAM3 requires a specific version of OpenCV (4.2). Install it from source as follows:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git unzip pkg-config \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libgtk-3-dev libcanberra-gtk3-module \
    libxvidcore-dev libx264-dev libxine2-dev \
    libv4l-dev v4l-utils \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libtbb2 libtbb-dev \
    libdc1394-22-dev \
    libopenexr-dev \
    libatlas-base-dev gfortran \
    python3-dev python3-numpy
```
Clone OpenCV and contrib repositories
```bash
cd ~
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd opencv
git checkout 4.2.0
cd ../opencv_contrib
git checkout 4.2.0
```

Build OpenCV
```bash
cd ~/opencv
mkdir build && cd build

cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=OFF \
      -D WITH_OPENGL=ON \
      -D BUILD_opencv_python3=ON \
      -D BUILD_EXAMPLES=OFF ..

make -j$(nproc)
sudo make install
sudo ldconfig
```
Verify installation
```bash
pkg-config --modversion opencv4
```
Expected output:
```
4.2.0
```


---

## 📂 Step 2: Set Up the Workspace

```bash
mkdir -p ~/orb_ws
cd ~/orb_ws
```

Clone ORB-SLAM3 **outside** the workspace:

```bash
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
```

Clone the ROS 2 wrapper inside your workspace:

```bash
cd ~/orb_ws
git clone https://github.com/TonyOx369/ORB_SLAM3_ROS2.git
```

Remove the existing build directories:

```bash
rm -rf /build /install /log
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
---

## 🔨 Step 5: Build ROS 2 Workspace

```bash
cd ~/orb_ws
colcon build
```

---

## ⚙️ Step 6: Create Camera Config File

Make a file `~/orb_ws/ZED_stereo_initial.yaml` with your respective camera intrinsics, extrinsics, and IMU parameters.

---

Rebuild:

```bash
cd ~/orb_ws
colcon build
```

---

## ▶️ Step 7: Run the System

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
https://github.com/user-attachments/assets/64952b97-d58a-4943-b276-0dbd26ef51d6





