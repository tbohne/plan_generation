# Plan Generation Node

**Simple ROS node that parses and provides handcrafted action plans as a service based on CSV data. Used in the context of my [**master's thesis**](https://github.com/tbohne/msc).**

## Plan Format (CSV)

Four actions are available in the scenario, and each line in the .csv file represents one action of the plan.
- `drive_to(lat, lng, theta)`
- `return_to_base`
- `charge`
- `scan`

## Sample Plan:
```
drive_to,52.32043739026998,8.153532860027937,0
scan
drive_to,52.320634371203766,8.153602612483242,0
scan
return_to_base
charge
```

## Run Node with Default Plan (`plan.csv`)

```
$ roslaunch plan_generation plan_generation.launch
```

## Run Node with Custom Plan

```
$ roslaunch plan_generation plan_generation.launch plan_path:=PATH_TO_PLAN
```

## Plan Retrieval Service

```
$ rosservice call /arox_planner/get_plan
```