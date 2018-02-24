add_executable( run_vo run_vo.cpp )
add_executable( run_be run_be.cpp )
target_link_libraries( run_vo slam )
target_link_libraries( run_be slam_be)