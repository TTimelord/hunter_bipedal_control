cmake_minimum_required(VERSION 3.16)

project(fsesdk)

find_package(catkin REQUIRED
        COMPONENTS
        )

catkin_package(
        INCLUDE_DIRS
                include
        LIBRARIES
                ${PROJECT_NAME}
        CATKIN_DEPENDS
                # dynamic_reconfigure
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# add_subdirectory(source build)
aux_source_directory(${PROJECT_SOURCE_DIR}/source dir_fse)
add_library(fsesdk SHARED ${dir_fse})



