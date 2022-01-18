# temoto_er_manager
External Resource Manager (ERM) allows to dynamically invoke programs (External Resources or ERs),
including ROS executables, ROS launch files and regular executables. Each requested program is a resource
that ERM is managing.

## Installation

Install ERM package
``` bash
cd catkin_ws/src
git clone https://github.com/temoto-framework/temoto_er_manager
```

## Examples
See [this example](https://github.com/temoto-framework/temoto_er_manager/blob/main/examples/test_er_client_node.cpp) of how ERM can be embedded in your code.