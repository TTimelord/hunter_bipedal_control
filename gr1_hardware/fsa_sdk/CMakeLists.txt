cmake_minimum_required(VERSION 3.13.1)

set(CMAKE_CXX_STANDARD 17)
project(fsasdk)


find_package(catkin REQUIRED
        COMPONENTS
        )


catkin_package(
        INCLUDE_DIRS
                include
                ThirdParty
                /usr/include/eigen3
        LIBRARIES
                ${PROJECT_NAME}
        CATKIN_DEPENDS
                # dynamic_reconfigure
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty)
include_directories(/usr/include/eigen3)

aux_source_directory(${PROJECT_SOURCE_DIR}/src dir_fsa)
add_library(fsasdk SHARED ${dir_fsa})

target_link_libraries(fsasdk pthread)

# install(TARGETS fsasdk DESTINATION ${PROJECT_SOURCE_DIR}/lib)
# set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

# install(DIRECTORY ${PROJECT_SOURCE_DIR}/include DESTINATION ${PROJECT_SOURCE_DIR}/install)