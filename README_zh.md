[English](README.md) | [中文](README_zh.md)

# OCP论文实验代码

共有4个包，其中
- `planning_node`包含了规划算法的代码。
- `ROSPlan`定义了一些msg和srv
- `tiago_demo_planning`是具体的案例，包含了两个场景的ExPDDL定义和各个行为的实现代码
- `tiago_dependencies`是tiago机器人的支持库，其中，`tiago_serving_demo/config`包含了world和map（场景1 tiago_5house_tables和场景2 tiago_5house_tables_2objs）

## 实验过程演示
[演示视频](https://www.bilibili.com/video/BV1NZ421b7BQ)

## 编译

### 运行环境
Ubuntu 20.04

### 克隆仓库

1. 下载代码。
2. 安装依赖的子模块
```shell
cd <this repo>
git submodule update --init --recursive
```

### 编译方法
使用`catkin build`进行编译。

## 运行

### 运行仿真环境
场景1
```shell
roslaunch tiago_serving_demo tiago_sim.launch world:=tiago_5house_tables
```
场景2
```shell
roslaunch tiago_serving_demo tiago_sim.launch world:=tiago_5house_tables_2objs
```

### 运行规划节点和行为节点

场景1

```shell
roslaunch tiago_demo_planning planning.launch
```

场景2

```shell
roslaunch tiago_demo_planning planning_2objs.launch
```

### 开始执行任务

```shell
time rosservice call /xyz_plan_dispatch/online_dispatch
```
调用服务的命令前面加上`time`，可以测量此命令执行的时长

### 查看任务规划时长

```shell
rostopic echo /xyz_knowledge_base/planning_time
```

单位为毫秒

## 代码细节

在所有行为的基类`planning_node::XYZActionInterface`中有一个函数`isSensingAction`，指示此行为是否为观察行为，实现观察行为时重载返回`true`，否则返回`false`，见`src/tiago_demo_planning/include/ActionInterface/WatchTable.h`第51行和`src/tiago_demo_planning/include/ActionInterface/Pick.h`第23行。

如果要实现一个观察行为，代码中需要根据观察结果手动更新知识库，见`src/tiago_demo_planning/src/ActionInterface/WatchTable.cpp`23-41行和`src/tiago_demo_planning/src/ActionInterface/LocateTwoObjs.cpp`29-67行。

## 目录结构

```
.
├── planning_node
│   ├── CMakeLists.txt
│   ├── files                       # 无用
│   ├── fmt-8.1.1                   # 第三方库，用于格式化打印
│   ├── include
│   ├── launch
│   ├── package.xml
│   └── src
│       ├── pddl_knowledge_base     # 知识库和规划器
│       │   ├── PDDLKnowledgeBase.cpp
│       │   ├── PDDLParser.cpp
│       │   ├── pddlparser-pp       # 子模块，ExPDDL解析器
│       │   └── pddl_planner.cpp    # 规划算法代码
│       ├── plan_dispatch           # 调度器
│       │   └── XYZPlanDIspatch.cpp
│       ├── planning_node           # 无用
│       ├── xyz_action_interface    # ActionInterface
│       │   └── XYZActionInterface.cpp
│       └── xyz_knowledge_base      # 无用
├── ROSPlan                         # 定义了一些消息和服务
└── tiago_demo_planning
    ├── config
    │   ├── laser_filter_config.yaml
    │   ├── map
    │   ├── rviz
    │   ├── waypoints.yaml          # 各导航目标点的坐标
    │   └── worlds
    ├── include
    ├── launch
    │   ├── planning_2objs.launch   # 场景2的launch文件
    │   ├── planning.launch         # 场景1的launch文件
    │   ├── sublaunch
    ├── pddl
    │   ├── domain_2objs.pddl       # 场景2的ExPDDL domain
    │   ├── problem_2objs.pddl      # 场景2的ExPDDL problem
    │   ├── domain.pddl             # 场景1的ExPDDL domain
    │   └── problem.pddl            # 场景1的ExPDDL problem
    ├── scripts
    │   └── run_demo.py             # 开始执行任务并记录任务完成时长
    └── src
        └── ActionInterface
            ├── Dummy.cpp
            ├── GotoWaypoint.cpp    # 导航行为代码
            ├── LocateTwoObjs.cpp   # 场景2中用到的寻找两个AruCo标的观察行为代码（locate_two_objs）
            ├── Pick.cpp            # 抓取行为代码
            ├── Place.cpp           # 放置行为代码
            └── WatchTable.cpp      # 寻找一个标是否在当前桌子上的观察行为代码（locate）
```

