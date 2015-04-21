##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_workflow_engine_blockly)

##############################################################################
# Catkin
##############################################################################
find_package(catkin REQUIRED cmake_nodejs_hook)

# This calls npm update only once.
catkin_npm_update_once()

# This calls npm update every catkin_make call
# catkin_npm_update_target()

catkin_package()
catkin_add_env_hooks(25.concert-workflow-engine-blockly SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
