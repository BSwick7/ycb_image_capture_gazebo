# For each ycb object, calculate the dimensions of the object using meshlab and save the dimensions to a yaml file in config/ycb_object_dimensions/{object_name}.yaml
# The dimensions are saved in the following format:
#     dimensions: [x, y, z]

import os
import subprocess
import yaml
import pymeshlab
import glob

import rospkg

# Standard main boilerplate
if __name__ == '__main__':
    package_name = 'ycb_image_capture_gazebo'
    try:
            model_path = rospkg.RosPack().get_path(package_name) + '/models/ycb/'
            model_xml = ''
    except rospkg.ResourceNotFound as e:
        print("Cannot find package [%s], check package name and that package exists, error message:  %s"%(package_name, e))
        
    # For each folder in the models/ycb directory, open the .sdf file with the same name as the folder
    # and extract the mesh file name from the sdf file
    # Then use meshlab to calculate the dimensions of the mesh file
    # Finally, save the dimensions to a yaml file in config/ycb_object_dimensions/{object_name}.yaml
    for folder in os.listdir(model_path):
        if os.path.isdir(model_path + folder):
            for file in os.listdir(model_path + folder):
                if file.endswith('.sdf'):
                    try:
                        with open(model_path + folder + '/' + file, 'r') as f:
                            model_xml = f.read()
                    except IOError as e:
                        print('Could not find or open model file %s, check model name and that model exists, I/O error message: %s', (model_path + folder + file), e)
                    break
            
            # Get the collision_mesh_uri file name to read into MeshLab
            saw_collision_tag = False
            for line in model_xml.split('\n'):
                if 'collision' in line:
                    saw_collision_tag = True
                if saw_collision_tag and 'uri' in line:
                    collision_mesh_uri = line.split('>')[1].split('<')[0]
                    print('Found collision mesh uri %s', collision_mesh_uri)
                    break
            
            # Load the mesh file into MeshLab and calculate the dimensions
            ms = pymeshlab.MeshSet()
            mesh_filename = model_path + collision_mesh_uri.split('//')[1]
            try:
                ms.load_new_mesh(mesh_filename)
                out_dict = ms.apply_filter('get_geometric_measures')
                model_x_size = out_dict['bbox'].dim_x()
                model_y_size = out_dict['bbox'].dim_y()
                model_z_size = out_dict['bbox'].dim_z()
            except IOError as e:
                print('Could not find or open mesh file %s, check model name and that model exists, I/O error message: %s', mesh_filename, e)
            except UnboundLocalError as e:
                print('Could not find or open mesh file %s, check model name and that model exists, error message: %s', mesh_filename, e)
            dimensions = [model_x_size, model_y_size, model_z_size]
            
            # Add the dimensions to a dict that will be written to a yaml file
            dimensions_dict = {'dimensions': dimensions}

            # Make the config/ycb_object_dimensions directory if it doesn't exist
            if not os.path.exists(rospkg.RosPack().get_path(package_name) + '/config/ycb_object_dimensions'):
                os.makedirs(rospkg.RosPack().get_path(package_name) + '/config/ycb_object_dimensions')

            # Write the dimensions to a yaml file   
            with open(rospkg.RosPack().get_path(package_name) + '/config/ycb_object_dimensions/' + folder + '.yaml', 'w') as f:
                yaml.dump(dimensions_dict, f)
            print('Saved dimensions for ' + folder + ' to ' + rospkg.RosPack().get_path(package_name) + '/config/ycb_object_dimensions/' + folder + '.yaml')
