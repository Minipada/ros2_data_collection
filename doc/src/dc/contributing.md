# Participating

## TLDR

| Event                               | What to do                                                                                                                                    |
| ----------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| Want to contribute                  | [Open a PR](https://github.com/Minipada/ros2_data_collection/pulls)                                                                           |
| Found a bug                         | [File a ticket on Github Issues](https://github.com/Minipada/ros2_data_collection/issues/new?assignees=&labels=bug&template=issues.md&title=) |
| Feature request                     | [Describe what you want on Github Discussions](https://github.com/Minipada/ros2_data_collection/discussions)                                  |
| Want to start a discussion          | [Start one on Github Discussions](https://github.com/Minipada/ros2_data_collection/discussions)                                               |
| Be aware of the ongoing development | Take a look at the [Github Project](https://github.com/users/Minipada/projects/1) and what is being worked on                                 |

## Contributing

### Feature requests

Since I want DC to be community driven, go to [Github discussions](https://ros2-data-collection.hellonext.co), start a discussion about a features you want to see and users will be able to vote for your it. Most requested features will have more attention than others.

### Found a bug?

If you find a problem, first search if an issue already exists. If a related issue doesn't exist, you can open a new issue using the [issue form](https://github.com/Minipada/ros2_data_collection/issues/new?assignees=&labels=bug&template=bug_report.md&title=).

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
Follow the steps to build your workspace and install dependencies in the [build section](./build.md)

Then, install the pre-commit hook:

```bash
$ pre-commit install
```

You are now ready to write some code, commit and follow the standards with the pre-commit hook.

#### Docs

To build the docs, install cargo:

```bash
$ sudo apt-get install cargo
```

Then install mdbook (the command line tool to create books with Markdown) and its plugins:

```bash
$ cargo install mdbook mdbook-admonish mdbook-linkcheck mdbook-mermaid
```

Start the doc locally with auto-refresh on edit:

```bash
$ export PATH=$PATH:$HOME/.cargo/bin
$ mdbook serve -p 8000 -n 0.0.0.0
```

And open [localhost:8000](http://localhost:8000)

Now that you open see the documentation locally, open the doc folder of the repository and edit the Markdown files you need. More about mdbook can be found [here](https://rust-lang.github.io/mdBook/guide/installation.html)


### Tests
TODO...
