
set(library_SOURCES
    math.cpp
    )

include_directories(math.hpp)
add_library(source SHARED ${library_SOURCES})
add_executable(main main.cpp)

set_target_properties(main PROPERTIES CMAKE_CXX_STANDARD_REQUIRED ON)
set_target_properties(main PROPERTIES CXX_STANDARD 17)