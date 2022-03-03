# TTC Radar Package

aev/aev_ws/src/ttcRadar_pkg

## Description

As a node to read/parse/public to ROS framedata serial mmWave_AOP_MistralRadar(IWR6843)


## Installation

### 1. Building workspace catkin

    mkdir -p ~/aev/aev_ws/src
    cd ~/aev/aev_ws
    catkin_make

### 2. Install serial port

##### Ubuntu 18.04 melodic

    sudo apt-get install ros-melodic-serial

##### Ubuntu 20.04 noetic

    cd ~/aev/aev_ws/src/
    git clone https://github.com/wjwwood/serial.git
    make
    make test
    make install

### 3. Download Radar package to aev_ws

##### Method 1: Download code Radar node

    cd ~/aev/aev_ws/src/
    git clone https://github.com/giangtin1920/ttcRadar_pkg.git

##### Method 2: Building radar package by hand

* Create radar package

      cd ~/aev/aev_ws/src
      catkin_create_pkg ttcRadar_pkg std_msgs rospy roscpp

* Create msg

      cd ttcRadar_pkg/
      mkdir msg
      cd msg
      touch ttcRadar_msg.msg
      nano ttcRadar_msg.msg  (add 3line to this file
      uint32 msg_counter
      bool isObject
      float32 distance
      )

* Open file CmakeLists.txt (`~/aev/aev_ws/src/ttcRadar_pkg`). Delete comment (`#`) like this

      find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        serial
        message_generation
      )

      add_message_files(
        FILES
        ttcRadar_msg.msg
      )

      generate_messages(
        DEPENDENCIES
        std_msgs
      )

      catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ttcRadar_pkg
        CATKIN_DEPENDS roscpp rospy std_msgs
        DEPENDS system_lib
      )

      include_directories(
        include
        ${catkin_INCLUDE_DIRS}
      )

    add to the endline:

      add_executable(
        ttcListener
        src/ttcListener.cpp
      )
      target_link_libraries(ttcListener ${catkin_LIBRARIES})
      add_dependencies(ttcListener ttcRadar_pkg_generate_messages_cpp)

      add_executable(
        ttcRadar 
        src/ttcRadar.cpp
        src/ttcRadar_Cfg.cpp
      )
      target_link_libraries(ttcRadar ${catkin_LIBRARIES})
      add_dependencies(ttcRadar ttcRadar_pkg_generate_messages_cpp)

* Open file package.xml (`~/aev/aev_ws/src/ttcRadar_pkg`). Delete comment (`<--`) like this

      <build_depend>message_generation</build_depend>
      <exec_depend>message_runtime</exec_depend>

* Copy all file executed (`ttcRadar.cpp`, ... ) to `~/aev/aev_ws/src/ttcRadar_pkg/src$`


## Usage

### Build workspace 

    cd ~/aev/aev_ws/
    catkin_make
    source ~/aev/aev_ws/devel/setup.bash
    . ~/aev/aev_ws/devel/setup.bash

### Run node radar in another Terminal

    rosrun ttcRadar_pkg ttcRadar
    
### Run node listener in another Terminal

    rosrun ttcRadar_pkg ttcListener
