# CLI tools
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
│ destinations     List destination plugins with their descriptions.                                                        │
│ measurements     List measurement plugins with their descriptions.                                                        │
╰───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────╯
```
