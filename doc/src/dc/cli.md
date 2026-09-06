# CLI tools

```admonish warning title="Known packaging gap: typer isn't declared"
`dc_cli/package.xml` doesn't declare a dependency on `typer` (the CLI framework this
tool is built on), and there is no `python3-typer` apt package for `rosdep` to resolve
either — a plain `rosdep install` + `colcon build` leaves `ros2 run dc_cli list_plugins`
failing with `ModuleNotFoundError: No module named 'typer'`. `pip install
--break-system-packages typer` (or add it to a `uv`-managed virtualenv) is the
workaround until the package declares it.
```

## List plugins
You can list available plugins by running the CLI tool:

```
ros2 run dc_cli list_plugins --help


 Usage: list_plugins [OPTIONS] COMMAND [ARGS]...

╭─ Options ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────╮
│ --install-completion          Install completion for the current shell.                                                   │
│ --show-completion             Show completion for the current shell, to copy it or customize the installation.            │
│ --help                        Show this message and exit.                                                                 │
╰───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────╯
╭─ Commands ────────────────────────────────────────────────────────────────────────────────────────────────────────────────╮
│ by-package       List plugins of a pluginlib file with their descriptions, by package name and filename.                  │
│ by-path          List plugins of a pluginlib file with their descriptions, by their path.                                 │
│ conditions       List condition plugins with their descriptions.                                                          │
│ measurements     List measurement plugins with their descriptions.                                                        │
╰───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────╯
```

```admonish info
There is no `destinations` command. Destinations are not pluginlib plugins in DC 2.0
(ADR-0003): the blessed types are listed in [Destinations](./destinations.md), and
everything else is reached through the passthrough.
```
