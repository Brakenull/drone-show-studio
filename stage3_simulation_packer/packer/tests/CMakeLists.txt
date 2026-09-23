add_executable(test_packer test_packer.cpp)
target_link_libraries(test_packer PRIVATE flight_packer_lib)
add_test(NAME test_packer COMMAND test_packer)

add_executable(test_spec_c test_spec_c.c)
target_include_directories(test_spec_c PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
add_test(NAME test_spec_c COMMAND test_spec_c)
