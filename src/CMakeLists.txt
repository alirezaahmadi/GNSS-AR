include_directories(${PROJECT_SOURCE_DIR})

add_executable(MainProject MainProject.cpp)
target_link_libraries(MainProject ${OpenCV_LIBS} Eigen3::Eigen)

