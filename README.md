This repo shows how the `ProximityRayPlugin` from [ARIAC2022](https://github.com/usnistgov/ARIAC/blob/12f48c48c2d36935cf54cdafa9376a5b75e147f4/nist_gear/src/ProximityRayPlugin.cc) can be migrated to the new Gazebo as a demonstration of migrating plugins from Gazebo-classic to the new Gazebo. See 
https://gazebosim.org/api/sim/7/migrationplugins.html and https://gazebosim.org/api/sim/7/ardupilot.html for more 
documentation.

### Gazebo-classic:
![gazebo-classic](https://user-images.githubusercontent.com/206116/200662881-f5259470-795e-492c-b78b-5140c1aad59f.gif)

### Gazebo (Garden):
![gazebo](https://user-images.githubusercontent.com/206116/200662883-4b4cc350-8a4d-4ee4-a972-98431977dfcb.gif)


## Build Instructions

### Gazebo-classic
```bash
$ sudo apt install libgazebo11-dev

$ cd gazebo-classic
$ cmake -H. -Bbuild
$ cmake --build build
$ export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$PWD/build # Set environment variable so that gazebo can find the plugin
$ gazebo --verbose -u test.sdf
```

**NOTE**: Gazebo-classic and Gazebo (Garden) cannot be installed at the same time. If you want to test running both, using
Docker is recommended.

### Gazebo (Garden)
```bash
$ sudo apt install gz-garden # Assuming you have the packages.osrfoundation.org repo already added. Otherwise, see https://gazebosim.org/docs/latest/install

$ cd gazebo
$ cmake -H. -Bbuild
$ cmake --build build
$ export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$PWD/build # Set environment variable so that gazebo can find the plugin
$ gz sim -v4 test.sdf
```
