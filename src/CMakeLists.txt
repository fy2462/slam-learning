add_library(slam SHARED camera.cpp config.cpp frame.cpp map.cpp mappoint.cpp visual_odometry.cpp)
target_link_libraries(slam ${THIRD_PARTY_LIBS})