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

### ros mynteye
https://slightech.github.io/MYNT-EYE-SDK/calibrate_with_opencv.html

###  ros注意事项
https://blog.csdn.net/weixin_42268975/article/details/80842175

###  ros tf
https://blog.csdn.net/sinat_27554409/article/details/48446553

###  ros z资料
http://www.voidcn.com/article/p-ehbmkpnz-bcg.html

### ros moveit
https://blog.csdn.net/lingchen2348/article/details/80300069</br>
https://blog.csdn.net/wxflamy/article/category/7410765/2</br>
https://blog.csdn.net/kalenee/article/category/7084979</br>
http://www.360doc.com/content/16/0825/16/7821691_585865831.shtml ---> 双臂</br>
https://hk.saowen.com/a/a67ac25f4bccd04a9c4d17bc5b6adebdbd3c1d23d01e2a5426c7d2ca9c72e925</br>
https://github.com/blodow/realtime_urdf_filter</br>
http://www.gaitech.net/news1.asp?sid=58</br>
ros暑期学校 : https://blog.csdn.net/ZhangRelay/article/details/80977751</br>
```使用moveit,在编译工作空间时遇到 /opt/ros/kinetic/include/moveit/macros/declare_ptr.h:52:16: error: ‘shared_ptr’ in namespace 
‘std’ does not name a template type typedef std::shared_ptr<Type> Name##Ptr
原因：不是使用c++11 ，
解决：https://github.com/ros-planning/moveit/issues/462 在相应功能包的CMakeLists.txt中添加 add_compile_options(-std=c++11)
     
``` 

### ros 软件包源
 https://blog.csdn.net/u013039705/article/details/80268575
 
### robotiq 
https://blog.csdn.net/sinat_37011812/article/details/79965957

### RPY与四元数的转换
ros : https://blog.csdn.net/flyfish1986/article/details/81634577 </br>
c++ :https://blog.csdn.net/zhuoyueljl/article/details/70789472?locationNum=4&fps=1

### ros book
https://github.com/wsnewman/learning_ros_kinetic

### ros手眼标定
https://blog.csdn.net/weixin_40799950/article/details/82594537

### ros有趣项目
https://github.com/BatyaGG/BCI-controlled-UR-manipulator</br>
https://github.com/davetcoleman/moveit_simple_grasps/<br>
https://github.com/andyzeng/apc-vision-toolbox</br>
https://github.com/flores-jacob/RoboND-Perception-Project</br>

### ros qt
https://www.ncnynl.com/archives/201701/1277.html</br>
https://blog.csdn.net/u014610460/article/details/79355533</br>

### ros修改moveit中的运动学插件
http://www.guyuehome.com/1921</br>

### ros 坐标系
https://blog.csdn.net/scliu12345/article/details/44658175</br>

### ros plugin
https://ros.guru/2018/02/12/10-useful-ros-tools/</br>
stereo_image_proc会自动根据camera校正结果发布修正后的图像和3D的topic.

### OROCOS
https://rtt-lwr.readthedocs.io/en/latest/tutos/orocos-tuto.html

### moveit平滑
https://www.codetd.com/article/869160</br>
http://www.guyuehome.com/752?replytocom=48315</br>
https://blog.csdn.net/weixin_28900531/article/details/79431055

### moveit 笛卡尔路径规划速度控制
https://github.com/epfl-lasa/ridgeback_ur5_controller

### ros kinect
https://blog.csdn.net/sunyoop/article/details/78517247

### ros ork
https://blog.csdn.net/zhuoyueljl/article/details/78965434</br>
https://blog.csdn.net/weixin_40799950/article/details/81911877</br>
https://blog.csdn.net/zhangrelay/article/details/77248232</br>
https://blog.techbridge.cc/2016/05/14/ros-object-recognition-kitchen/</br>
http://wg-perception.github.io/ork_tutorials/index.html

### ros marker 生成器
http://keystone.umd.edu/html/markergen.html

###  ros csdn博客
https://blog.csdn.net/ZhangRelay

### ros修改 轨迹速度？？
https://answers.ros.org/question/264957/moveit-trajectory-only-providing-translation-not-velocity-or-acceleration/</br>
https://groups.google.com/forum/#!topic/moveit-users/7n45S8DUjys

### robot operation system coobook 
https://books.google.com/books?id=68RiDwAAQBAJ&pg=PA359&lpg=PA359&dq=multi_dof_joint_trajectory&source=bl&ots=DyShiHlalU&sig=IKXDa8ik0tuyolDevSITBeqSfkw&hl=en&sa=X&ved=2ahUKEwjKkafyiJLeAhXCj1QKHWwnBF84ChDoATAIegQIAhAB#v=onepage&q=multi_dof_joint_trajectory&f=false

### ros倒入scene文件
https://answers.ros.org/question/287394/moveit-import-scene-from-text-automatically/</br>
下载dae模型:http://models.gazebosim.org/

### ros功能包下载
https://mirror.vtti.vt.edu/packages.ros.org/ros/ubuntu/pool/main/r/

### ply to pcd
pcl_ply2pcd -format 0 cloud.ply cloud.pcd

### DEX-NET
http://berkeleyautomation.github.io/dex-net/

### tabletop_object
http://wiki.ros.org/tabletop_object_detector

### cob_object_detect
http://wiki.ros.org/cob_object_detection

### ros 简书
https://www.jianshu.com/p/de1654877f50

### 四元数与RPY在线转换
https://quaternions.online/ </br>
https://www.andre-gaschler.com/rotationconverter/


#### 综上所述，机器人作为ModbusTCP服务器，上位机通过502端口可以控制机器人的所有IO;上位机可以通过30001或30002或30003端口远程下载程序到机器人；上位机通过29999端口可以远程控制程序运行状态；上位机通过30003端口可以实时得到机器人的状态信息。也就是说，利用UR机器人开放的基于TCP/IP协议的端口，可以制作自己的Polyscope软件，实现上位机的远程控制。

### ros 博客园
https://www.cnblogs.com/shangchele/p/7328187.html

### vs2015 配置pcl
https://blog.csdn.net/bbhhtt/article/details/81156437</br>
https://bbs.csdn.net/topics/390930955?page=1</br>
https://blog.csdn.net/Gordon_Wei/article/details/85236064

### window c++ ur control
https://blog.csdn.net/zmdsjtu/article/details/53432177(修改为  movel(p[x,y,z,rx,ry,rx],a,v))
