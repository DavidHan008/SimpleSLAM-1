## SimpleSLAM

### Algorithms
The basic point of SLAM ( for either LiDAR or Visual) is merging adjacent frames. Hence we follow the simplest framework of SLAM to build such a SimpleSLAM project just for practice:

- Step 1: Read in frames and complete preprocessing
- Step 2: Extract features from multiple frames
- Step 3: Match similar features and calculate rigid transformation
- Step 4: Merge multiple frames into the global coordinate system
- Step 5: Write the merged pointcloud into .pcd files

### Dependencies
You should have Eigen3 and PCL in your system before starting this project.

```bash
  sudo apt install libeigen3-dev
```

Use PCL from ROS or install PCL apis according to the official tutorial.

### Compile 
Run following scripts when you are at the root directory of the project.

```bash
  mkdir build
  cp -r ./pcd/* ./build
  cd build
  cmake ..
  make
```

### Execution
```bash
  # Now you are in the ./build directory
  ./2dslam base.pcd curr.pcd
```

### Problems
There are still severe problems:
- The feature extraction may not be correct (hence the feature matching)
- The object-oriented programming in this project should be improved
- The consistency of the code style should be guaranteed
