# temoto_process_manager
Process Manager allows to dynamically invoke programs (processes),
including ROS executables, ROS launch files and regular executables. Each requested program is a resource
that Process Manager is managing.

## Installation

Install the Process Manager package
``` bash
cd catkin_ws/src
git clone https://github.com/temoto-framework/temoto_process_manager
```

## Examples
See [this example](https://github.com/temoto-framework/temoto_process_manager/blob/ros2_devel/examples/test_pm_client_node.cpp) of how Process Manager can be embedded in your code.


In one terminal start TeMoto Process Manager
``` bash
ros2 run temoto_process_manager process_manager_node
```

In a separate terminal, start the test client node
``` bash
ros2 run temoto_process_manager pm_client_node
```


