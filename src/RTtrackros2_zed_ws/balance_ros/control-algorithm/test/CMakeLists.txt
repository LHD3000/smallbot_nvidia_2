find_package(GTest REQUIRED)

add_executable(lqr_test lqr.cpp)
target_link_libraries(lqr_test GTest::GTest control_algorithm)
add_test(NAME lqr_test COMMAND lqr_test)

#add_executable(mpc_test mpc.cpp)
#target_link_libraries(mpc_test GTest::GTest control_algorithm)
#add_test(NAME mpc_test COMMAND mpc_test)