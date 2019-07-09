# cmake_minimum_required(VERSION 3.8)

set(CMAKE_RELEASE_DIR "${CMAKE_SOURCE_DIR}/bin/release")
# debug binary folder
set(CMAKE_DEBUG_DIR "${CMAKE_SOURCE_DIR}/bin/debug")

enable_testing()
find_package(GTest REQUIRED)
# include(GoogleTest)
# include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(bachelor_test test.cpp)

set_target_properties(bachelor_test PROPERTIES CMAKE_CXX_STANDARD_REQUIRED ON)
set_target_properties(bachelor_test PROPERTIES CXX_STANDARD 17)
target_link_libraries(bachelor_test GTest::GTest GTest::Main source)
gtest_discover_tests(bachelor_test)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
	set_target_properties(bachelor_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_DEBUG_DIR})
	set_target_properties(bachelor_test PROPERTIES OUTPUT_NAME bachelor${BUILD_PREFIX})
else()
	set_target_properties(bachelor_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RELEASE_DIR})
endif()

