# Plan Generation Node

**Simple node that parses and provides handcrafted plans as service based on csv data.**

## Plan Format (CSV)

There are four available actions and each line in the .csv file represents one action of the plan.
- **drive_to**: `name,lat,lng,orientation`
    - e.g. `drive_to,52.32043739026998,8.153532860027937,0`
- **return_to_base**: `name`, e.g. `return_to_base`
- **charge**: `name`, e.g. `charge`
- **scan**: `name`, e.g. `scan`

-----
*Example Plan:*  
`drive_to,52.32043739026998,8.153532860027937,0`  
`scan`  
`drive_to,52.320634371203766,8.153602612483242,0`  
`scan`  
`return_to_base`  
`charge`  

-----

## Usage

- The csv-plan to be used can be configured via the parameter `plan_path` in the launch file.  
- launch plan generator / provider: `roslaunch plan_generation plan_generation.launch`
