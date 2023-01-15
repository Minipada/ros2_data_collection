# File - Fluent Bit

## Description

The file output plugin allows to write the data received through the input plugin to file. See [fluent bit page](https://docs.fluentbit.io/manual/pipeline/outputs/file) for more information.

## Parameters

| Parameter  | Description                                                  | Type | Default      |
| ---------- | ------------------------------------------------------------ | ---- | ------------ |
| **path**   | Topic from where odom is used to know if the robot is moving | str  | "$HOME/data" |
| **file**   | Speed threshold used in the hysteresis                       | str  | "data"       |
| **format** | Counter to know if the robot is moving                       | str  | "plain"      |
| **mkdir**  | Create directory if missing                                  | str  | "true"       |
