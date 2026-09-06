# Beginner

Everything prints to your terminal: nothing to install, start or clean up afterwards, and
nothing to go and look at in another tool.

**Prerequisites**: a [built and sourced](../setup.md#build) workspace, and nothing else — no
containers, no databases. **Roughly 5 minutes each**, 15 for the Turtlebot3 one including
simulator startup.

| Title                                                | Description                                                                            | Also needs                                                |
| ------------------------------------------------------ | ----------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| [Uptime](./uptime_stdout.md)                          | Collect how long the system has been running and print it on Stdout. Minimal example    | —                                                          |
| [Group memory and uptime](./memory_uptime_stdout.md) | Collect both memory and uptime and group them in a dictionary                           | —                                                          |
| [Turtlebot3 Stdout](./tb3_stdout.md)                  | Collect command velocity, map, position and speed and print it in stdout                | The Nav2 Turtlebot3 simulation (see [Setup](../setup.md)) |
