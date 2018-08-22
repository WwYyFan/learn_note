## Ros Note

#### ros 安装 Hash检验和不符
 ```
     更换源  https://blog.csdn.net/CattonBoom/article/details/80118177
 ```

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

头文件不用添加add_executable  add_dependecies  target_link_libraries
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

#### 引用其他库文件（例子：引用halcon库)
```
   link_directories(
  /opt/halcon/lib/x64-linux
  /opt/halcon/lib/x64-linux/qt
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
  /opt/halcon/include
  /opt/halcon/include/cpp
  /opt/halcon/include/halconcpp
  /opt/halcon/include/halconc
  /opt/halcon/include/hdevengine
  /opt/halcon/include/hdevengine10
  /opt/halcon/include/hlib
)

file(GLOB LIBS /opt/halcon/lib/x64-linux/lib*.so)

FOREACH(src ${LIBS})
   MESSAGE(${src})
   link_libraries(${src})
endforeach()

add_executable(halcon_test
  src/halcon_test.cpp
)
add_dependencies(halcon_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(halcon_test
  ${catkin_LIBRARIES} 
)
```

#### rosrun package a.py
Python文件 记得修改权限后  才能rosrun


#### 在 node 元素中使用 output 属性：
output=”screen”
带这个属性启动的节点会将标准输出信息显示在终端的窗口中，而不会保存在日志文件中。这也 解释 了为什么这个带有output=”screen”的节点（node） 的日志文件在上面日志文件列表中丢失的原因。 

###  ros双目标定
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 --no-service-check --approximate=0.1 right:=/right/camera/image_raw left:=/left/camera/image_raw right_camera:=/right/camera left_camera:=/left/camer
