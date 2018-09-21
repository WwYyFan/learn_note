## 使用moveit！控制真实机械臂（3）——修改moveit配置文件来控制真实机械臂
要想使用moveit来控制真实机械臂，我们需要修改配置文件夹下的几个文件，因为默认生成的moveit配置文件中，所使用的部分参数是针对虚拟机械臂的，你可以在rviz环境下观察模型的运动，但真正的控制信号并不会发出来。具体要修改以下几个地方：

---------------------

#### 1、demo.launch文件中参数fake_execution的值改为false
```
<!--此段代码来自moveit配置文件demo.launch-->
<!-- Run the main MoveIt executable without trajectory execution (we do not have controllers configured by default) -->
  <include file="$(find aubo_i5_moveit_config)/launch/move_group.launch">
    <arg name="allow_trajectory_execution" value="true"/>
    <arg name="fake_execution" value="false"/> //请看这个参数
    <arg name="info" value="true"/>
    <arg name="debug" value="$(arg debug)"/>
  </include>
```
通过参数的名字也很好理解，就是启用真实机械臂执行方式。接下来，demo.launch文件会启动move_group.launch文件，这是下一个要修改的文件。

#### 2、修改moveit_controller_manager参数。
move_group.launch文件中，moveit_controller_manager在选择参数值时，“unless”前面那个velue值要修改，写一个自己机器人名称作为前缀
```
<!-- Trajectory Execution Functionality -->
  <include ns="move_group" file="$(find aubo_i5_moveit_config)/launch/trajectory_execution.launch.xml" if="$(arg allow_trajectory_execution)">
    <arg name="moveit_manage_controllers" value="true" />
    <arg name="moveit_controller_manager" value="aubo_i5" unless="$(arg fake_execution)"/>  //请看这里
    <arg name="moveit_controller_manager" value="fake" if="$(arg fake_execution)"/>
  </include>
 ```
这段代码要看懂，moveit_controller_manager的参数是有选择的，要么等于“aubo_i5”（我使用的名称），要么等于“fake”，这要取决于后面的参数fake_execution，而这个参数我们上一步已经改为了false，即当前moveit_controller_manager应该等于“aubo_i5”,接下来，这个参数会传递给trajectory_execution.launch.xml文件，该文件中有下面这句话
```
<include file="$(find aubo_i5_moveit_config)/launch/$(arg moveit_controller_manager)_moveit_controller_manager.launch.xml" />
```

按照原先的设置，本应该启动fake_moveit_controller_manager.launch.xml，你可以去moveit配置文件夹下去找，这个文件是存在的，而现在，一个叫做aubo_i5_moveit_controller_manager.launch.xml将会被启动，而这个文件moveit也应该已经帮你创建好了，当你使用先前的moveit向导加载机器人模型时，机器人模型中写明的机器人名称（name属性），就会作为前缀写入这个文件的文件名，所以，“aubo_i5”这个名称源自于你模型文件里写明的机器人名称，请前后保持统一，理解各个文件之间的调用关系。

#### 3、修改aubo_i5_moveit_controller_manager.launch.xml文件
双击打开这个文件，发现里面只写了一对launch标签，基本算是一个空文件，说明moveit早就帮你留好了位置，等着你来填充，填什么呢？我们可以参考moveit官网中关于controllers Configuration Tutorial这一部分，里面清楚的给出了一份填写建议：
```
<launch>
<!-- Set the param that trajectory_execution_manager needs to find the controller plugin -->
<arg name="moveit_controller_manager" default="moveit_simple_controller_manager/MoveItSimpleControllerManager" />
<param name="moveit_controller_manager" value="$(arg moveit_controller_manager)"/>
<!-- load controller_list -->
<rosparam file="$(find my_robot_name_moveit_config)/config/controllers.yaml"/>
</launch>
```
请将上面这段代码复制到你的对应文件中，将my_robot_name_moveit_config修改为你自己的moveit配置文件目录名称。

#### 4、创建controllers.yaml配置文件
上一步的代码最后一句指向了config目录下一个叫做controllers.yaml的文件，这个文件很重要，决定了你所使用的moveit控制器的基本参数。我们打开配置文件夹中的config目录，发现只有fake_controllers.yaml,所以，现在要做的就是复制一份这个文件，然后将名字改为controllers.yaml，打开这个新文件，将文件修改成如下形式：
```
controller_list:
  - name: "aubo_i5"
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    joints:
      - shoulder_joint
      - upperArm_joint
      - foreArm_joint
      - wrist1_joint
      - wrist2_joint
      - wrist3_joint
```
你的文件很可能写的不是这样，但理解参数含义就好

name:这里你可以写一个与你机器人相关的名称，方便你使用

action_ns:follow_joint_trajectory ，follow_joint_trajectory 是后续action的名称的一部分，有关action的内容在下一章里，将来你需要填一个action的名称，这个名称就在这个配置文件里确定了，就是name/action_ns所代表的对应字符串的组合。

type: FollowJointTrajectory ，这个类型是ros下的自带action类型，是一种控制机械臂运动轨迹的数据结构，请原样填写，将来你可能还想控制手抓之类的结构，所填写的类型都是不一样的。

joints：这里是你机器人（机械臂）的关节名称，这些名称源自你的机器人模型文件，我们的controllers.yaml文件复制于fake_controllers.yaml文件，这部分应该是自动生成的。

到此为止，moveit配置文件中所有应该修改的文件就修改完了，moveit已经准备好了将规划的轨迹以action的形式发送给真实机械臂，后面的内容将是如何利用好action这里的信息，实现真正的机械臂控制。


#### 运行
```
1. ur_bringup.launch
2. demo.launch
```

---------------------

**本文来自 爱学习的草莓熊 的CSDN 博客 ，全文地址请点击：https://blog.csdn.net/lingchen2348/article/details/80300069?utm_source=copy 
