## *********************************************************
##
## File autogenerated for the explore_server package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'name': 'Default', 'type': '', 'state': True, 'cstate': 'true', 'id': 0, 'parent': 0, 'parameters': [{'name': 'timeout', 'type': 'double', 'default': 30.0, 'level': 0, 'description': 'Explore Timeout(s)', 'min': 10.0, 'max': 100.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'min_frontier_size', 'type': 'double', 'default': 0.75, 'level': 0, 'description': 'Min Frontier Size(m)', 'min': 0.5, 'max': 2.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'visualize', 'type': 'bool', 'default': True, 'level': 0, 'description': 'Is Visualize?', 'min': False, 'max': True, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}, {'name': 'frontier_type', 'type': 'str', 'default': 'centroid', 'level': 0, 'description': 'Select Frontier Type', 'min': '', 'max': '', 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': "{'enum': [{'name': 'centroid', 'type': 'str', 'value': 'centroid', 'srcline': 7, 'srcfile': '/home/qyd/dx/one_ws/src/my_controllers/config/Explore.cfg', 'description': 'Use Centroid', 'ctype': 'std::string', 'cconsttype': 'const char * const'}, {'name': 'initial', 'type': 'str', 'value': 'initial', 'srcline': 8, 'srcfile': '/home/qyd/dx/one_ws/src/my_controllers/config/Explore.cfg', 'description': 'Use Initial', 'ctype': 'std::string', 'cconsttype': 'const char * const'}, {'name': 'middle', 'type': 'str', 'value': 'middle', 'srcline': 9, 'srcfile': '/home/qyd/dx/one_ws/src/my_controllers/config/Explore.cfg', 'description': 'Use Middle', 'ctype': 'std::string', 'cconsttype': 'const char * const'}], 'enum_description': 'An enum to set frontier type'}", 'ctype': 'std::string', 'cconsttype': 'const char * const'}, {'name': 'potential_scale', 'type': 'double', 'default': 3.0, 'level': 0, 'description': 'Distance Weight', 'min': -inf, 'max': inf, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'gain_scale', 'type': 'double', 'default': 1.0, 'level': 0, 'description': 'Gain Weight', 'min': -inf, 'max': inf, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'information_scale', 'type': 'double', 'default': 1.2, 'level': 0, 'description': 'Information Weight', 'min': -inf, 'max': inf, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'information_r', 'type': 'double', 'default': 3.0, 'level': 0, 'description': 'Information Radius', 'min': -inf, 'max': inf, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}], 'groups': [], 'srcline': 246, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT', 'parentclass': '', 'parentname': 'Default', 'field': 'default', 'upper': 'DEFAULT', 'lower': 'groups'}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

Explore_centroid = 'centroid'
Explore_initial = 'initial'
Explore_middle = 'middle'
