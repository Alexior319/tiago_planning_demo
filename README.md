[English](README.md) | [中文](README_zh.md)

# Experimental code for the paper OCP

Here is the code for the paper: `OCP: An Online Contingent Planning Method for Robot Tasks in Unknown Environments`.

There are total 4 packages，including:
- `planning_node` code for the planning algorithm
- `ROSPlan` some ROS msgs and services
- `tiago_demo_planning` The exact case, including two scenarios and the ExPDDL files and implentations for each action
- `tiago_dependencies` dependencies for tiago robot

## Experimental video
[video](https://youtu.be/MLVHmmNfv30)

## Compilation

### Environment
- Ubuntu 20.04
- ROS Noetic

### Clone this repo
1. Download this repo.
2. Install dependencies
```shell
cd <this repo>
git submodule update --init --recursive
```

3. compile
```shell
catkin build
```

## Run

### Run the simulated environment
Scenario 1
```shell
roslaunch tiago_serving_demo tiago_sim.launch world:=tiago_5house_tables
```
Scenario 2
```shell
roslaunch tiago_serving_demo tiago_sim.launch world:=tiago_5house_tables_2objs
```

### Run the planning node and dispatching node

Scenario 1

```shell
roslaunch tiago_demo_planning planning.launch
```

Scenario 2

```shell
roslaunch tiago_demo_planning planning_2objs.launch
```

### Start executing

```shell
time rosservice call /xyz_plan_dispatch/online_dispatch
```

### View the total time used

```shell
rostopic echo /xyz_knowledge_base/planning_time
```

time unit: `ms`


## Directory structure

```
.
├── planning_node
│   ├── CMakeLists.txt
│   ├── files                       
│   ├── fmt-8.1.1                  
│   ├── include
│   ├── launch
│   ├── package.xml
│   └── src
│       ├── pddl_knowledge_base     # KB and the planner
│       │   ├── PDDLKnowledgeBase.cpp
│       │   ├── PDDLParser.cpp
│       │   ├── pddlparser-pp       # submodule, ExPDDL parser
│       │   └── pddl_planner.cpp    # planner algorithm
│       ├── plan_dispatch           # dispatcher
│       │   └── XYZPlanDIspatch.cpp
│       ├── planning_node           
│       ├── xyz_action_interface    # ActionInterface
│       │   └── XYZActionInterface.cpp
│       └── xyz_knowledge_base      
├── ROSPlan                         # ROS messages and services
└── tiago_demo_planning
    ├── config
    │   ├── laser_filter_config.yaml
    │   ├── map
    │   ├── rviz
    │   ├── waypoints.yaml          
    │   └── worlds
    ├── include
    ├── launch
    │   ├── planning_2objs.launch   # launch file of Scenario 1
    │   ├── planning.launch         # launch file of Scenario 2
    │   ├── sublaunch
    ├── pddl
    │   ├── domain_2objs.pddl       # ExPDDL domain of Scenario 2
    │   ├── problem_2objs.pddl      # ExPDDL problem of Scenario 2
    │   ├── domain.pddl             # ExPDDL domain of Scenario 1
    │   └── problem.pddl            # ExPDDL problem of Scenario 1
    ├── scripts
    │   └── run_demo.py             # Start executing and record time used
    └── src
        └── ActionInterface
            ├── Dummy.cpp
            ├── GotoWaypoint.cpp    
            ├── LocateTwoObjs.cpp   
            ├── Pick.cpp            
            ├── Place.cpp           
            └── WatchTable.cpp      
```
