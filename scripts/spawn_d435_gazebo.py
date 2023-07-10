#! /usr/bin/env python3

# ROS Imports
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from tf.transformations import quaternion_from_euler

def spawn_camera_urdf(camera_name, camera_position, camera_rpy):
    """ Spawns the camera in Gazebo using the urdf file
    Args:
        camera_name (str): Name of the camera
    """

    # Verify that the realsense2_description package is installed
    package_name = 'realsense2_description'
    try:
        model_path = rospkg.RosPack().get_path(package_name) + '/urdf'
        model_xml = ""
        rospy.loginfo('Package %s found', package_name)
    except rospkg.ResourceNotFound as e:
        rospy.logerr('Package %s not found, verify that the realsense-ros package has been installed, error message: %s'%(package_name, e))
        return
    
    # Generate the urdf file using the xacro command
    import subprocess
    try:
        model_xml = subprocess.check_output('xacro ' + model_path + '/test_d435_camera.urdf.xacro static:=true', shell=True).decode('utf-8').replace('\n', '')
        rospy.loginfo('Urdf file generated successfully')
    except subprocess.CalledProcessError as e:
        rospy.logerr('Error while generating the urdf file, error message: %s'%(e))
        return 
    
    # Create the spawn_urdf_model service proxy
    try:
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        spawn_urdf_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        rospy.loginfo('Service /gazebo/spawn_urdf_model found')
    except rospy.ROSException as e:
        rospy.logerr('Error while creating the spawn_urdf_model service proxy, error message: %s'%(e))
        return
    
    # Convert the roll, pitch, yaw to quaternion
    camera_orientation = quaternion_from_euler(camera_rpy[0], camera_rpy[1], camera_rpy[2])

    # Create the spawn_urdf_model request
    request = SpawnModelRequest()
    request.model_name = camera_name
    request.model_xml = model_xml
    request.robot_namespace = ''
    request.initial_pose.position.x = camera_position[0]
    request.initial_pose.position.y = camera_position[1]
    request.initial_pose.position.z = camera_position[2]
    request.initial_pose.orientation.x = camera_orientation[0]
    request.initial_pose.orientation.y = camera_orientation[1]
    request.initial_pose.orientation.z = camera_orientation[2]
    request.initial_pose.orientation.w = camera_orientation[3]
    request.reference_frame = 'world'

    # Try to spawn the camera
    spawn_camera_urdf_response = SpawnModelResponse()
    try:
        spawn_camera_urdf_response = spawn_urdf_model(request)
        rospy.loginfo('Service /gazebo/spawn_urdf_model called')
    except rospy.ServiceException as e:
        rospy.logerr('Error while spawning the camera, error message: %s'%(e))
        return
    
    # Log the success
    if spawn_camera_urdf_response.success:
        rospy.loginfo('Camera %s spawned successfully', camera_name)
    else:
        rospy.logerr('Error while spawning the camera, error message: %s'%(spawn_camera_urdf_response.status_message))
        return

# Main function boilerplate
if __name__ == '__main__':
    # Initalize the ROS node
    rospy.init_node('spawn_d435_gazebo', anonymous=False)
    rospy.loginfo('Node %s started', rospy.get_name())

    # Get the camera name
    camera_name = rospy.get_param('~camera_name', 'd435')

    # Get the camera position and roll, pitch, yaw
    camera_position = rospy.get_param('~camera_position', [0.0, 0.0, 0.0])
    camera_rpy = rospy.get_param('~camera_rpy', [0.0, 0.0, 0.0])

    # Try to spawn the camera
    try:
        spawn_camera_urdf(camera_name, camera_position, camera_rpy)
    except rospy.ROSInterruptException:
        pass
