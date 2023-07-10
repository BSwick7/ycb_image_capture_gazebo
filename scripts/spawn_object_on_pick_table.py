#! /usr/bin/env python3

# Python Imports
import numpy as np
import glob
import yaml

# ROS Imports
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel, SpawnModelResponse, SpawnModelRequest, GetWorldProperties, GetModelState, GetModelStateRequest, GetModelStateResponse
from geometry_msgs.msg import Pose

def spawn_sdf_model(object_name):
    """ Spawns an sdf model in Gazebo in a specific position and orientation
    Args:
        object_name (string): Name of the object to spawn
    Returns:
        bool: True if the object was successfully spawned, False otherwise
    """
    # Open the model file
    package_name = 'ycb_image_capture_gazebo'
    try:
            model_path = rospkg.RosPack().get_path(package_name) + '/models/ycb/'
            model_xml = ''
    except rospkg.ResourceNotFound as e:
        rospy.logerr("Cannot find package [%s], check package name and that package exists, error message:  %s"%(package_name, e))
        return False
   
    # Given the object name, find the model folder in model_path/models/ycb/ where the model folder has the format {:03}_object_name
    # where {:03} is a 3 digit number with leading zeros
    # Then, go into the folder and open the model.sdf file
    try:
        model_folder = glob.glob(model_path + '*' + object_name)[0] + '/'
        rospy.loginfo('Found model folder %s', model_folder)
    except UnboundLocalError as e:
        rospy.logdebug('Could not find model folder %s, error message: %s', model_folder, e)
        return False

    # Read the XML data for the model
    xml_filename = model_folder + object_name + '.sdf' 
    try: 
        with open(xml_filename, 'r') as xml_file:
            xml_data = xml_file.read()
            model_xml = xml_data.replace('\n', '')
            # Read the xml file to get the location of the collision mesh uri
            saw_collision_tag = False
            for line in xml_data.split('\n'):
                if 'collision' in line:
                    saw_collision_tag = True
                if saw_collision_tag and 'uri' in line:
                    collision_mesh_uri = line.split('>')[1].split('<')[0]
                    rospy.loginfo('Found collision mesh uri %s', collision_mesh_uri)
                    break
    except IOError as e:
        rospy.logerr('Could not find or open model file %s, check model name and that model exists, I/O error message: %s', xml_filename, e)
        return False
    except UnboundLocalError as e:
        rospy.logdebug('Could not find or open model file %s, check model name and that model exists, error message: %s', xml_filename, e)
        return False
    
    # Try and load the dimensions from the {:03d}_{object_name}.yaml file in config/ycb_object_dimensions/
    try:   
        yaml_filename = rospkg.RosPack().get_path(package_name) + '/config/ycb_object_dimensions/' + model_folder.split('/')[-2] + '.yaml'
        with open(yaml_filename, 'r') as yaml_file:
            yaml_data = yaml_file.read()
            dimensions = yaml.safe_load(yaml_data)['dimensions']
            rospy.loginfo('Found yaml file %s', yaml_filename)
    except IOError as e:
        rospy.logerr('Could not find or open yaml file %s, check model name and that model exists, I/O error message: %s', yaml_filename, e)

    
    # Adjust the spawn bounds based on the object dimensions to prevent it from spawning off of the table
    spawn_bounds = 0.4572
    spawn_x_min = -spawn_bounds + dimensions [0] / 2.0
    spawn_x_max = spawn_bounds - dimensions[0] / 2.0
    spawn_y_min = -spawn_bounds + dimensions[1] / 2.0
    spawn_y_max = spawn_bounds - dimensions[1] / 2.0

    # If the object size is larger than the spawn bounds, then don't spawn it and log an error
    if spawn_x_min > spawn_x_max or spawn_y_min > spawn_y_max: 
        rospy.logerr('Object %s is too large to spawn on the table', object_name)
        return False

    # Check if the object is already spawned and change the object name if it is
    object_count = 0
    try:
        rospy.wait_for_service('/gazebo/get_world_properties')
        get_world_properties_prox = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        get_world_properties_res = get_world_properties_prox()
        for model_name in get_world_properties_res.model_names:
            if object_name in model_name:
                object_count += 1
        if object_count > 0:
            object_name = object_name + '_' + str(object_count)
            rospy.loginfo('Object with name %s already exists, changing object name to %s', model_name, object_name)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr('Error while waiting for service %s to be available. Error message: %s', service_name, e)
        return False
    
    # Create the request object
    spawn_model_req = SpawnModelRequest()

    # Fill the request object with the data
    table_z_height = 0.9144
    spawn_model_req.reference_frame = 'world'
    spawn_model_req.model_name = object_name
    spawn_model_req.model_xml = model_xml
    spawn_model_req.initial_pose = Pose()
    spawn_model_req.initial_pose.orientation.x = 0.0
    spawn_model_req.initial_pose.orientation.y = 0.0
    spawn_model_req.initial_pose.orientation.z = 0.0
    spawn_model_req.initial_pose.orientation.w = 0.0
    spawn_model_req.initial_pose.position.x = np.random.uniform(spawn_x_min, spawn_x_max)
    spawn_model_req.initial_pose.position.y = np.random.uniform(spawn_y_min, spawn_y_max)
    spawn_model_req.initial_pose.position.z = table_z_height
    rospy.loginfo('Object %s initial spawn location at (%f, %f)', spawn_model_req.model_name, spawn_model_req.initial_pose.position.x, spawn_model_req.initial_pose.position.y)

    # Check if the model will spawn inside of another object
    # If it does, then try to spawn it again
    # If it doesn't after 5 tries, then log an error and return False
    spawn_attempts = 0
    max_spawn_attempts = 5
    expected_models = ['wc3_table', 'ground_plane']
    scene_models_x_loc = []
    scene_models_y_loc = []
    try:
        rospy.loginfo('Checking for objects on the table that could cause spawn location collisions...')
        rospy.wait_for_service('/gazebo/get_world_properties')
        rospy.loginfo('Service /gazebo/get_world_properties is now available.')
        get_world_properties_prox = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        get_world_properties_res = get_world_properties_prox()
        possible_collision_models = list(set(get_world_properties_res.model_names) - set(expected_models))
        try:
            rospy.wait_for_service('/gazebo/get_model_state')
            rospy.loginfo('Service /gazebo/get_model_state is now available.')
            get_model_state_prox = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            for model_name in possible_collision_models:
                get_model_state_req = GetModelStateRequest()
                get_model_state_req.model_name = model_name
                get_model_state_res = get_model_state_prox(get_model_state_req)
                if get_model_state_res.success:
                    scene_models_x_loc.append(get_model_state_res.pose.position.x)
                    scene_models_y_loc.append(get_model_state_res.pose.position.y)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('Error while waiting for service /gazebo/get_model_state to be available. Error message: %s', e)
            return False
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr('Error while waiting for service /gazebo/get_world_properties to be available. Error message: %s', e)
        return False
    
    if len(scene_models_x_loc) != 0:
        rospy.loginfo('Found %d models on table, checking for collisions with spawn location', len(scene_models_x_loc))
        invalid_spawn = True
    else:
        rospy.loginfo('Found no models on table, skipping collision check')
        invalid_spawn = False

    while invalid_spawn and spawn_attempts < max_spawn_attempts:
        dist_from_spawn_x = np.abs(np.array(scene_models_x_loc) - spawn_model_req.initial_pose.position.x)
        dist_from_spawn_y = np.abs(np.array(scene_models_y_loc) - spawn_model_req.initial_pose.position.y)
        # Check if the spawn location is inside of any of the models on the table
        if dist_from_spawn_x.min() < dimensions[0] and dist_from_spawn_y.min() < dimensions[1]:
            rospy.loginfo("Object %s will spawn inside of an object, trying again...", object_name)
            spawn_model_req.initial_pose.position.x = np.random.uniform(spawn_x_min, spawn_x_max)
            spawn_model_req.initial_pose.position.y = np.random.uniform(spawn_y_min, spawn_y_max)
        else:
            invalid_spawn = False
        spawn_attempts += 1
    
    if invalid_spawn:
        rospy.logerr('Object %s could not be spawned after %d attempts', object_name, max_spawn_attempts)
        return False
    
    rospy.loginfo('Object %s final spawn location at (%f, %f)', object_name, spawn_model_req.initial_pose.position.x, spawn_model_req.initial_pose.position.y)
    service_name = '/gazebo/spawn_sdf_model'
    try:
        # Wait for the service to be available
        rospy.loginfo('Waiting for service %s to be available...', service_name)
        rospy.wait_for_service(service_name)
        rospy.loginfo('%s service is now available.', service_name)
        
        # Create the connection to the service
        spawn_model_prox = rospy.ServiceProxy(service_name, SpawnModel)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr('Error while waiting for service %s to be available. Error message: %s', service_name, e)
        return False
    
    spawn_model_res = SpawnModelResponse()
    try:
        # Call the service
        rospy.loginfo('Calling %s service...', service_name)
        spawn_model_res = spawn_model_prox(spawn_model_req)
        rospy.loginfo('%s service successfully called.', service_name)
        
        # Check the response
        if spawn_model_res.success:
            rospy.loginfo('%s successfully spawned.', spawn_model_req.model_name)
        else:
            rospy.logerr('%s could not be spawned. Error message: %s', spawn_model_req.model_name, spawn_model_res.status_message)
            return False
    except rospy.ServiceException as e:
        rospy.logerr('Error when calling %s service: %s', service_name, e)
        return False
    

if __name__ == '__main__':
    # Initalize the ROS node
    rospy.init_node('spawn_object_on_pick_table', anonymous=False)
    rospy.loginfo('Node %s started', rospy.get_name())

    # Load the model name from the roslaunch file parameter
    object_name = rospy.get_param('~object_name')

    try:
        spawn_sdf_model(object_name)
    except rospy.ROSInterruptException:
        pass

