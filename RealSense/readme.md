# Intel RealSense Camera with Python on Linux

This repository contains a guide and example code to help you set up and use an Intel RealSense camera with Python on a Linux system.

## Prerequisites

- Ubuntu 18.04 or later
- Python 3.6 or later
- Intel RealSense camera (e.g., D415, D435)

## Installation

### Step 1: Install the Intel RealSense SDK

1. Update the package lists:
    ```sh
    sudo apt-get update
    ```

2. Install the required packages:
    ```sh
    sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
    sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
    ```

3. Clone the Intel RealSense library:
    ```sh
    git clone https://github.com/IntelRealSense/librealsense.git
    cd librealsense
    ```

4. Install Udev rules:
    ```sh
    sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```

5. Build and install the library:
    ```sh
    mkdir build
    cd build
    cmake ../ -DCMAKE_BUILD_TYPE=Release
    make -j4
    sudo make install
    ```

### Step 2: Install the Python Wrapper

Ensure you have `pip` installed and install the **pyrealsense2** package.
```sh
pip install pyrealsense2
```
