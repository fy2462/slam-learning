add_library(slam SHARED camera.cpp config.cpp frame.cpp map.cpp mappoint.cpp visual_odometry.cpp g2o/g2o_types.cpp)
add_library(slam_be SHARED backend_optimization.cpp g2o/g2o_types.cpp beop/BALProblem.cpp camera.cpp config.cpp flags/command_args.cpp)
target_link_libraries(slam ${THIRD_PARTY_LIBS})
target_link_libraries(slam_be ${THIRD_PARTY_LIBS})