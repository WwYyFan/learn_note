##Ros Note

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
