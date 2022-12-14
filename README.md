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
See [this example](https://github.com/temoto-framework/temoto_process_manager/blob/main/examples/test_pm_client_node.cpp) of how Process Manager can be embedded in your code.