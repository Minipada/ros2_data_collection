# FAQ

## Can I use DC without Fluent Bit backend?
It is possible to use DC without Fluent Bit as backend. There is an example with the RCL destination, which subscribes to a ROS topic and prints it in the console.

Still, a lot of work would be required to disable Fluent Bit and keep similar features (backpressure handling, data persistence and measurements and destinations available)
