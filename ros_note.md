## Ros Note

#### ros包 引用头文件
 1. 引用本包头文件
 ```
 CMakeList.txt
 ~~~~~~~~~~~~~~~~~~~~~~
 catkin_package(INCLUDE_DIRS include)
    
 include_directories(
    include ${catkin_INCLUDE_DIRS}
    include
    ${catkin_INCLUDE_DIRS}
)
 ```
 
 2. A包 引用 B包头文件、
 - A package
 ```
 CMakeList.txt
 ~~~~~~~~~~~~~~~~~~~~~~
 find_package(catkin REQUIRED COMPONENTS B)
 
 include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

 package.xml
 ~~~~~~~~~~~~~~~~~~~~~~
 <depend>B</depend>
 ```
 - B package
  ```
  CMakeList.txt
 ~~~~~~~~~~~~~~~~~~~~~~
   catkin_package(
  INCLUDE_DIRS include
  LIBRARIES B
  )
  
  include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
)
  
  add_library(B src/we_move.cpp) 
  add_dependencies(B ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  ```

 *注 ： 头文件不加add_executable 和 add_dependencies


#### roslaunch 的启动参数 arg, 它与launch中的param, rosparam有很大不同。
 parameter是运行中的ROS系统使用的数值，存储在参数服务器（parameter server）中，每个活跃的节点都可以通过 ros::param::get 或NodeHandle中函数来获取parameter的值，用户也可以通过rosparam来获得parameter的值。
而argument只在启动文件内才有意义他们的值是不能被节点直接获取的。
