# relase binary folder
set(CMAKE_RELEASE_DIR "${CMAKE_SOURCE_DIR}/bin/release")
# debug binary folder
set(CMAKE_DEBUG_DIR "${CMAKE_SOURCE_DIR}/bin/debug")


find_package(Doxygen)
option(BUILD_DOC "Build documentation" OFF)


include_directories(${CMAKE_SOURCE_DIR}/visualizer/include
                    ${CMAKE_SOURCE_DIR}/include
                    ${CMAKE_SOURCE_DIR}/3rd-library)
                    # ../lib/Sophus
                    # ../lib/easylogging
                    # ../include)


set(LIBRARY_SOURCES
    matrix.cpp
    cell_info.cpp
    square_grid_graph.cpp
    shortest_path.cpp
    a_star.cpp
    cost.cpp
    up_hill_cost.cpp
    down_hill_cost.cpp
    heuristic.cpp
    )


# include_directories(math.hpp)
add_library(source SHARED ${LIBRARY_SOURCES})

set_target_properties(source PROPERTIES CMAKE_CXX_STANDARD_REQUIRED ON)
set_target_properties(source PROPERTIES CXX_STANDARD 17)

add_executable(bachelor main.cpp)
target_link_libraries (bachelor visualizer source)

set_target_properties(bachelor PROPERTIES CMAKE_CXX_STANDARD_REQUIRED ON)
set_target_properties(bachelor PROPERTIES CXX_STANDARD 17)


if(CMAKE_BUILD_TYPE STREQUAL Debug)
	set_target_properties(bachelor PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_DEBUG_DIR})
	set_target_properties(bachelor PROPERTIES OUTPUT_NAME bachelor${BUILD_PREFIX})
else()
	set_target_properties(bachelor PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RELEASE_DIR})
endif()

if (BUILD_DOC)
   set(DOXYGEN_OUTPUT_FOLDER ${CMAKE_SOURCE_DIR}/docs/output)

    # set input and output files
    set(DOXYGEN_IN ${CMAKE_SOURCE_DIR}/docs/doxyfile.in)
    set(DOXYGEN_OUT ${DOXYGEN_OUTPUT_FOLDER}/doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    # message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating HTML Documentation with Doxygen"
        VERBATIM )
endif (BUILD_DOC)