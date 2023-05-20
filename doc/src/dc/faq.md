# FAQ

## I can't find a measurement and/or destination I need

Even though measurements and destinations will constantly be added in the future, the current focus is on getting feedback, fixing bugs, documentation and reach a minimum test coverage.

Create a feature request in [Github Discussions](https://github.com/Minipada/ros2_data_collection/discussions/categories/ideas-and-feature-requests)...or better, write your plugin and do a Pull Request.

## Can I use DC without Fluent Bit backend?
It is possible to use DC without Fluent Bit as backend. There is an example with the RCL destination, which subscribes to a ROS topic and prints it in the console.

Still, a lot of work would be required to disable Fluent Bit and keep similar features (backpressure handling, data persistence and measurements and destinations available)

## How can I add a Fluent Bit plugin?

Currently, from my knowledge, it is only possible to load a plugin using the C api which loads a shared library. Because of that, I expect more languages can be used, such as Rust, the only requirement is to use the bindings. For measurements, it is not a problem, we can write a node in any language publishing on a StringStamped message and use the measurement plugin with the same name.

Destination plugins can be written in C or Go. You can find examples in the fluent_bit_plugins package.

## How can I use a Python API with Fluent Bit

Adding a destination plugin which uses a Python library is not cleared. We have not yet solved this problem and would welcome one, since many API libraries are available in this language.

Note for the courageous ones: I tried once to create python-C bindings of Fluent Bit, by creating .h files like in the [fluent-bit-go](https://github.com/fluent/fluent-bit-go) and use ctypesgen(https://github.com/ctypesgen/ctypesgen), like this:

```bash
ctypesgen \
  --allow-gnu-c \
  --output-language=py32 \
  ~/ws/fluent-bit/include/fluent-bit.h \
  -I./lib/msgpack-c/include/ \
  -I./lib/monkey/include \
  -I./build/lib/monkey/include/monkey \
  -I./lib/cfl/include \
  -I./lib/cfl/lib/xxhash \
  -I./include \
  -I./lib/flb_libco \
  -I./lib/c-ares-1.19.0/include \
  -I./lib/cmetrics/include \
  -I./lib/ctraces/include \
  -I./lib/ctraces/lib/mpack/src \
  -o fluent_bit.py
```

It creates a base for the bindings. While I suppose it is possible to generate them, I'm puzzled as how this will be loaded by Fluent Bit afterwards.

Whatever you write, feel free to make a Pull Request and share your work!

## My group data is not published on the group topic

This may happen for different reasons:

1. The group node is not started, be sure it is (ros2 node list), you will need to enable it in your launch file or using the `group_node:=true` when launching the bringup
2. Data is not being published on all topics it subscribes to (use ros2 topic echo on each to ensure that). Currently, if all topics don't publish, there is no way to ignore that and still send the message partially full (see [here](https://answers.ros.org/question/410138/is-it-possible-to-drop-or-keep-message-in-approximatetimesynchronizer/)). Help is welcome here :)
