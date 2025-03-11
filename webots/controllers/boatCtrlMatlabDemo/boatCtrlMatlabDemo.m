% MATLAB controller for Webots
% File:          boatCtrlMatlabDemo.m
% Date:
% Description:
% Author:
% Modifications:

timestep = wb_robot_get_basic_time_step();

% boatNode = wb_supervisor_node_get_from_def('boat');
% positionField = wb_supervisor_node_get_field(boatNode, 'translation');
% wb_supervisor_field_set_sf_vec3f(positionField, [0, 20, 0.35]);
leftMotor = wb_robot_get_device('left_propeller');
rightMotor = wb_robot_get_device('right_propeller');
uiopen('./boatCtrlDemo.slx', 1);
