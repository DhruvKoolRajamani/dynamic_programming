project(hw7)

add_executable(${PROJECT_NAME}_problem_1 problem_1.cpp)
add_executable(${PROJECT_NAME}_problem_2 problem_2.cpp)
add_executable(${PROJECT_NAME}_problem_3 problem_3.cpp)

target_link_libraries(${PROJECT_NAME}_problem_1
  sub::envs
)

target_link_libraries(${PROJECT_NAME}_problem_2
  sub::envs
)

target_link_libraries(${PROJECT_NAME}_problem_3
  sub::envs
)