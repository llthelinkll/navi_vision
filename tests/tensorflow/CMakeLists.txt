cmake_minimum_required (VERSION 2.6)
project (orb_test)

set(PROJECT_DIR ../..)
find_package( OpenCV REQUIRED )

include_directories(
  ${PROJECT_DIR}/include
)

add_library(${PROJECT_NAME} SHARED
  ${PROJECT_DIR}/src/ORB_Extractor.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
)

add_executable(tensorflow tensorflow.cpp)
target_link_libraries(tensorflow ${PROJECT_NAME})
