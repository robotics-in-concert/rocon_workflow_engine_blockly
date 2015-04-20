##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_workflow_engine_blockly)

##############################################################################
# Catkin
##############################################################################
find_package(catkin REQUIRED)

#catkin_npm_update() Under development
# set(NPM_UPDATE_BIN "/usr/bin/npm")
# set(NPM_UPDATE "update")

# message(STATUS "    ... Installing Node dependencies")
# message(STATUS "    ... CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
# execute_process(COMMAND  ${NPM_UPDATE_BIN} ${NPM_UPDATE}
# WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
# RESULT_VARIABLE NPM_RESULT
# OUTPUT_VARIABLE NPM_VARIABLE)

# message(STATUS "    ... Done.. [${NPM_RESULT}]: ${NPM_VARIABLE}")

catkin_package()
catkin_add_env_hooks(25.concert-workflow-engine-blockly SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
