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

target_compile_options(${PROJECT_NAME} PUBLIC
  -g
)

add_executable(kiti eightpoints.cpp)
target_link_libraries(kiti ${PROJECT_NAME})
