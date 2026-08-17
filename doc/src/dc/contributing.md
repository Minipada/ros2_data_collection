# Participating

## TLDR

| Event                               | What to do                                                                                                                                    |
| ----------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| Want to contribute                  | [Open a PR](https://github.com/Minipada/ros2_data_collection/pulls)                                                                           |
| Found a bug                         | [File a ticket on Github Issues](https://github.com/Minipada/ros2_data_collection/issues/new?assignees=&labels=bug&template=issues.md&title=) |
| Found a vulnerability               | [Report it privately](https://github.com/Minipada/ros2_data_collection/security/advisories/new), never as a public issue                      |
| Feature request                     | [Describe what you want on Github Discussions](https://github.com/Minipada/ros2_data_collection/discussions)                                  |
| Want to start a discussion          | [Start one on Github Discussions](https://github.com/Minipada/ros2_data_collection/discussions)                                               |
| Be aware of the ongoing development | Take a look at the [Github Project](https://github.com/users/Minipada/projects/1) and what is being worked on                                 |

## Contributing

### Feature requests

Since I want DC to be community driven, go to [Github discussions](https://github.com/Minipada/ros2_data_collection/discussions), start a discussion about a features you want to see and users will be able to vote for your it. Most requested features will have more attention than others.

### Found a bug?

If you find a problem, first search if an issue already exists. If a related issue doesn't exist, you can open a new issue using the [issue form](https://github.com/Minipada/ros2_data_collection/issues/new?assignees=&labels=bug&template=bug_report.md&title=).

### Found a vulnerability?

Do not open a public issue, discussion or pull request. Report it through
[GitHub private vulnerability reporting](https://github.com/Minipada/ros2_data_collection/security/advisories/new)
instead; the [security policy](https://github.com/Minipada/ros2_data_collection/blob/jazzy/SECURITY.md)
states the supported branches, the response targets and what is in scope.

### General guidelines

You can contribute to the source code with Pull Requests, for example:

* To fix a typo you found on the documentation.
* To propose new documentation sections.
* To fix an existing issue/bug.
  * Make sure to add tests.
* To add a new feature.
  * Make sure to add tests.
  * Make sure to add documentation if it's relevant.


### Setup environment
#### ROS
Follow the steps to build your workspace and install dependencies in the [setup section](./setup.md)

Then install the git hook. `.pre-commit-config.yaml` is run by
[prek](https://prek.j178.dev), a single-binary reimplementation of pre-commit:

```bash
uv tool install prek   # or: curl -LsSf https://prek.j178.dev/install.sh | sh
prek install
```

You are now ready to write some code, commit and follow the standards with the git hook.
To run every hook over the whole tree the way CI does:

```bash
prek run --all-files --skip build-doc   # drop the --skip to check the docs build too
```

#### Docs

To build the docs, install cargo:

```bash
sudo apt-get install cargo
```

Then install mdbook (the command line tool to create books with Markdown) and its plugins:

```bash
cargo install mdbook mdbook-admonish mdbook-linkcheck mdbook-mermaid
```

Start the doc locally with auto-refresh on edit:

```bash
export PATH=$PATH:$HOME/.cargo/bin
mdbook serve -p 8000 -n 0.0.0.0
```

And open [localhost:8000](http://localhost:8000)

Now that you open see the documentation locally, open the doc folder of the repository and edit the Markdown files you need. More about mdbook can be found [here](https://rust-lang.github.io/mdBook/guide/installation.html)


### Declaring plugin parameters

Measurement and Condition plugins (`dc_measurements/plugins/{measurements,conditions}/`)
declare their own parameters in `onConfigure()`. Always do this through
`dc_util::get_*_type_param()` (`dc_util/include/dc_util/node_utils.hpp`) — never call
`declare_parameter` or `nav2_util::declare_parameter_if_not_declared` directly. One call
both declares and reads the value, and exits with a clear `RCLCPP_FATAL` if it can't be
retrieved, instead of a hand-rolled declare/get/try-catch block per parameter:

```cpp
// Mandatory (no default; fatal if not overridden):
cam_name_ = dc_util::get_str_type_param(node, measurement_name_, "cam_name");

// Optional, with a default:
polling_interval_ = dc_util::get_int_type_param(node, measurement_name_, "polling_interval", 1000);
```

`plugin_name`/`measurement_name_`/`condition_name_` is the namespace prefix — the helper
declares and reads `"<plugin_name>.<param_name>"`. Available types: `str`, `str_array`,
`bool`, `bool_array` (mandatory only), `int`, `int_array` (mandatory only), `double`,
`double_array` (mandatory only). `measurement_server.cpp`/`group_server.py`-level
parameters that have no plugin namespace use the equivalent `dc_util::get_str_param()` /
`get_str_array_param()` (no `plugin_name` argument).

This is a single-source-of-truth convention deliberately, not just a style preference: see
[ADR-0008](https://github.com/Minipada/ros2_data_collection/blob/jazzy/docs/adr/0008-dc-util-owns-parameter-declaration.md)
for why `nav2_util` stays a dependency and `dc_util` wraps it rather than replacing it.

`dc_group` (Python) has no plugins and no equivalent wrapper — `group_server.py` declares
each of its parameters exactly once via plain `self.declare_parameter(...)`, which is
sufficient there (see the ADR).

### Tests
TODO...
