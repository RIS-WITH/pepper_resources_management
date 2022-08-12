# Pepper resources management

[![Dependency Status][Dependency-Image]][Dependency-Url]

This is a meta package grouping necessary components to manage physical resources of the Pepper robot.

It is composed of:

 - A **head** manager
 - An **arm** manager
 - A **base** manager
 - A synchronizer

To run it, simply launch:

```bash
roslaunch pepper_resources_synchronizer pepper_resources_synchronizer.launch
```

In addition, a node to track a frame is available. You can run at as follow:

```bash
rosrun pepper_head_human_following pepper_head_human_following_node
```

[Dependency-Image]: https://img.shields.io/badge/dependencies-resource_management-1eb0fc.svg
[Dependency-Url]: https://github.com/RIS-WITH/resource_management