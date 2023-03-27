# File - Fluent Bit

## Description

The file output plugin allows to write the data received through the input plugin to file. See [fluent bit page](https://docs.fluentbit.io/manual/pipeline/outputs/file) for more information.

## Parameters

| Parameter           | Description                                                                                                              | Type                         | Default           |
| ------------------- | ------------------------------------------------------------------------------------------------------------------------ | ---------------------------- | ----------------- |
| **file**            | Set file name to store the records. If not set, the file name will be the tag associated with the records.               | str                          | N/A (Optional)    |
| **format**          | The format of the file content. See also [Format](https://docs.fluentbit.io/manual/pipeline/outputs/file#format) section | str(out_file,plain,csv,ltsv) | "out_file"        |
| **mkdir**           | Recursively create output directory if it does not exist. Permissions set to 0755.                                       | bool                         | true              |
| **path**            | Directory path to store files. If not set, Fluent Bit will write the files on it's own positioned directory              | str                          | "$HOME/data/"     |
| **delimiter**       | The character to separate each data. Default to ',' if format=csv, '\t'(TAB) if format=ltsv, '' else                     | str                          | '' or ',' or '\t' |
| **label_delimiter** | The character to separate label and the value. Default: ':'. Used for ltsv                                               | str                          | '.'               |

## Node configuration

```yaml
...
flb_file:
  plugin: "dc_destinations/FlbFile"
  inputs: ["/dc/group/cameras"]
  file: "data"
  format: "out_file"
  mkdir: true
  path: "$HOME/data/"
...
```
