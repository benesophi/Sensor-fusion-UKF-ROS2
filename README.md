
# Sensor Fusion UKF - ROS 2

## Description
This repository contains the software architecture for a sensor fusion system developed using **ROS 2 Foxy**. The project implements an **Unscented Kalman Filter (UKF)** to estimate vehicle state (position and orientation) by fusing high-frequency IMU data with low-frequency GNSS data.

This implementation was developed as part of an undergraduate research project at the University of São Paulo (USP).

## System Architecture
The fusion logic is divided into two main nodes to ensure modularity and clarity:

* **`data_extraction_node` (Preprocessing & Coordinate Transformation):**
    This node implements the first stage of the pipeline. It synchronizes data from multiple sensors and performs the coordinate transformation from the Geodetic system (Latitude, Longitude, Altitude) to a local Cartesian system (East, North, Up - ENU).

* **`ukf_node` (UKF Application):**
    The core algorithm. It functions to fuse the high-frequency data (prone to drift) from the IMU with the absolute positioning data from the GNSS. This generates a continuous and robust state estimation.

## Dependencies & Prerequisites

### System Requirements
* **ROS 2 Foxy** (Ubuntu 20.04)
* **Python 3**

### Python Libraries
The necessary Python dependencies can be installed via `pip`:

```bash
pip install numpy scipy filterpy pyproj pandas matplotlib
```

### ⚠️ Note on External Assets

  * **Proprietary Drivers:** The `carina` package (used for GNSS interfacing in the original setup) is proprietary and **is not included** in this repository.
  * **Datasets:** The raw `rosbag` files are not included due to size limits.
  * **Launch File Note:** The provided launch file is configured to play the rosbag automatically. If you do not have the specific bag file, you may need to comment out that section in the launch file or provide your own data source.

## Usage

### Data Input (Rosbag)
The filter relies on reading sensor data played back from a **rosbag**. The system requires the bag file to contain specific topics for:
* **GNSS Receivers** 
* **IMU** 

**Note:** This implementation is not exclusive to a specific dataset. You can run the filter on any rosbag containing the compatible topics.

### Configuration
To use a different dataset, simply open the launch file and modify the `bag_path` variable to point to your desired `.db3` or `.mcap` file.

### Execution

The entire fusion pipeline (preprocessing + UKF + rosbag playback) is orchestrated by a single launch file.

To execute:

```bash
ros2 launch kalmanfilter kalmanfilter.launch
```

## Author

**Sophia de Souza Nobre Benevides**
Mechatronics Engineering Student at University of São Paulo (EESC-USP).

-----

*This project is for educational and research purposes.*

