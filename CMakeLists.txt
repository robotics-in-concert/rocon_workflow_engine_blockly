##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_workflow_engine)

##############################################################################
# Catkin
##############################################################################
find_package(catkin REQUIRED)

#catkin_npm_update() Under development

catkin_package()
catkin_add_env_hooks(25.concert-workflow-engine-blockly SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)