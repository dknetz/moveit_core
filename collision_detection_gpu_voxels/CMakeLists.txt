set(MOVEIT_LIB_NAME moveit_collision_detection_gpu_voxels)

add_library(${MOVEIT_LIB_NAME}
  src/collision_robot_gpu_voxels.cpp
  src/collision_world_gpu_voxels.cpp
)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_collision_detection ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)
  
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_gpu_vexels_collision_detection test/test_gpu_voxels_collision_detection.cpp)
  target_link_libraries(test_gpu_vexels_collision_detection  ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})
endif()
