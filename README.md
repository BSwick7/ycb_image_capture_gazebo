# ycb_image_capture_gazebo

## Installation
1. Clone the repo into the src folder of your ROS workspace
2. Clone the following repos into the src folder:
`rickstaa/realsense_gazebo_plugin`
`BSwick7/realsense_ros`
3. Download the YCB object models
4. Run the YCB python processing script at:
`sea-bass/ycb-tools`
5. Copy the model files into a new folder `models/ycb`

## Main functions
1. `pick_table.launch`: spawns a pick table based on the aims-robot-all workcell 3. Loads the world environment from the pick_table.world file in the `worlds` folder
2. `spawn_d435_gazebo.launch`: spawns an Intel RealSense d435 camera in Gazebo. The position of the camera is loaded from the `config` folder in Cartesian world coordinates and then converted to a Quarternion in the script.
3. `spawn_object_on_pick_table.launch`: spawns an object in a random location on the table accounting for the dimensions of the table and possible collisions. Loads the models from the `models/ycb` folder