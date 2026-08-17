# Overview

## Description
A condition enables or disables one or multiple measurements to be published and thus collected. We could for example enable collecting camera images only when a robot is stopped:

Each condition is enabled or disabled through a pluginlib plugin. It has these configuration parameters:

```admonish note title="A Condition is a level, not an edge"
A Condition gates collection for as long as its predicate holds, so it can only give you data
from the moment it became true onwards. To capture what happened *before* an event, use a
[Trigger](./triggers.md): it is built out of these same Condition plugins but fires once on
the false→true edge, releasing a window a Measurement had already buffered.
```

## Available plugins:

| Name                                                     | Description                                      |
| -------------------------------------------------------- | ------------------------------------------------ |
| [Robot moving](./conditions/moving.md)                   | Robot is moving                                  |
| [Bool equal](./conditions/bool_equal.md)                 | Value of a boolean key is equal to               |
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
