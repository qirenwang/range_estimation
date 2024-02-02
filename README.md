# Range Estimation

## Project Description
This is a project related to electric vehicles range estimation. This module is part of our VCBench, which is a platform for testing and benchmarking. The project is based on ROS2 Humble. We created this package under the ROS2 Humble workspace. If you haven't created a workspace before, please check the [ROS2 Humble tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html). We reproduce part of the work from "Electric Vehicles Driving Range and Energy Consumption Investigation: A Comparative Study of Machine Learning Techniques." Thanks to the authors for releasing their codes and providing related algorithms.

## Setup
- Ensure your pip tool is up-to-date using `pip install --upgrade pip`.
- Install any missing packages using `pip install package_name`.
- Before running the module, source the setup file with `source install/local_setup.bash`, and then use `colcon build` in the terminal.
- Run modules using `ros2 run module_name node_name`.

## Usage

### Running the Publisher
To play rosbag, use the following command:
```bash
ros2 bag play rosbag2_2024_01_28-18_04_12_0.db3
```

### Running the Subscriber
To start the subscriber node, use the following command:
```bash
ros2 run range_estimation subscriber
```


## Citation
Find the related published conference paper [here](https://ieeexplore.ieee.org/abstract/document/9066042).

```
@inproceedings{amirkhani2019electric,
  title={Electric Vehicles Driving Range and Energy Consumption Investigation: A Comparative 
  Study of Machine Learning Techniques},
  author={Amirkhani, Abdollah and Haghanifar, Arman and Mosavi, Mohammad R},
  booktitle={2019 5th Iranian Conference on Signal Processing and Intelligent Systems (ICSPIS)},
  pages={1--6},
  year={2019},
  organization={IEEE}
}
```
