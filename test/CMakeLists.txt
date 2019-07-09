cmake_minimum_required(VERSION 3.8)

# include(CTest)
enable_testing()
find_package(GTest REQUIRED)
# include(GoogleTest)
# include_directories(${GTEST_INCLUDE_DIRS})
include_directories(SYSTEM ../src)

add_executable(main_test math-test.cpp)
#${GTEST_BOTH_LIBRARIES}

set_target_properties(main_test PROPERTIES CMAKE_CXX_STANDARD_REQUIRED ON)
set_target_properties(main_test PROPERTIES CXX_STANDARD 17)

# GTest::Main
target_link_libraries(main_test GTest::GTest GTest::Main source)
# add_test(AllTests main_test)
# gtest_add_tests(TARGET main_test)
gtest_discover_tests(main_test)

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)
