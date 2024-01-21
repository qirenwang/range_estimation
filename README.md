# range_estimation

## Project Description
This is a project related to electric vechicles range estimation. This module is part of our VCBench which is a platform for testing and benchmark. The project based on ROS2 humble. We created this package under this ROS2 humble workspace. If you didn't create a workspace before, please check [ROS2 humble tutoral](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html). We reproduce part of work from Electric Vehicles Driving Range and Energy Consumption Investigation: A Comparative Study of Machine Learning Techniques. Thanks authors release their codes and provide related algorithms.

## Setup
Please use `pip install --upgrade pip`, make sure your pip tool is up to date. Is some packages are missing, please use `pip install package_name`.

Every time you should `source install/local_setup.bash`, and then `colcon build` in the terminal before your type in `ros2 run module_name node_name`



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
