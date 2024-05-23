cmake_minimum_required(VERSION 3.16)

project(fsesdk)

catkin_package(
        INCLUDE_DIRS
                include
        LIBRARIES
                ${PROJECT_NAME}
        CATKIN_DEPENDS
                # dynamic_reconfigure
)

add_subdirectory(source build)



