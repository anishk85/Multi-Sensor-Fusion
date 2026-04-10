cmake_minimum_required(VERSION 3.10.2)
project(husarion_ugv_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes models rviz urdf worlds
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
        scripts/diff_drive_xbox_teleop.py
        scripts/quantum_filter_node.py
        DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/setup_envs.sh.in")

ament_package()
