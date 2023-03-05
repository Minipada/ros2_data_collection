# Overview

## Description
A condition enables or disables one or multiple measurements to be published and thus collected. We could for example enable collecting camera images only when a robot is stopped:

Each condition is enabled or disabled through a pluginlib plugin. It has these configuration parameters:

## Available plugins:

| Name                                                     | Description                                      |
| -------------------------------------------------------- | ------------------------------------------------ |
| [Robot moving](./conditions/moving.md)                   | Robot is moving                                  |
| [Bool equal](./conditions/bool_equal.md.md)              | Value of a boolean key is equal to               |
| [Double equal](./conditions/double_equal.md)             | Value of a double key is equal to                |
| [Double inferior](./conditions/double_inferior.md)       | Value of a double key is inferior to             |
| [Double superior](./conditions/double_superior.md)       | Value of a double key is superior to             |
| [Exist](./conditions/exist.md)                           | Key exists                                       |
| [Integer equal](./conditions/integer_equal.md)           | Value of an integer key is equal to              |
| [Integer inferior](./conditions/integer_inferior.md)     | Value of an integer key is inferior to           |
| [Integer superior](./conditions/integer_superior.md)     | Value of an integer key is superior to           |
| [List bool equal](./conditions/list_bool_equal.md)       | Value of a list of boolean key is equal to       |
| [List double equal](./conditions/list_double_equal.md)   | Value of a list of double key is equal to        |
| [List integer equal](./conditions/list_integer_equal.md) | Value of a list of integer key is equal to       |
| [List string equal](./conditions/list_string_equal.md)   | Value of a list of string key is equal to        |
| [Same as previous](./conditions/same_as_previous.md)     | Value of the key is the same as the previous one |
| [String match](./conditions/string_match.md)             | Value of a string key matches the regex of       |
