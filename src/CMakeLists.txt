add_library(slam SHARED camera.cpp config.cpp frame.cpp map.cpp mappoint.cpp visual_odometry.cpp g2o_types.cpp)
target_link_libraries(slam ${THIRD_PARTY_LIBS})