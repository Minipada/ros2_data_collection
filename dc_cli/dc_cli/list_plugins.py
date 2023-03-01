import pathlib
import xml.etree.ElementTree as ET

import typer
from ament_index_python.packages import get_package_share_directory

app = typer.Typer()


def print_plugins(path: pathlib.Path):
    if not path.exists():
        raise ValueError(f"{path} does not exist")
    tree = ET.parse(path.as_posix())
    root = tree.getroot()

    plugins = []

    for library in root:
        plugins.append(
            {"description": library[0][0].text.strip(), "class": library[0].attrib["name"]}
        )

    plugins = sorted(plugins, key=lambda i: (i["class"]))

    for plugin in plugins:
        print(f"{plugin['class']}: {plugin['description']}")


@app.command()
def by_package(
    package: str = typer.Option(..., help="Package to get the pluginlib file from"),
    filename: str = typer.Option(..., help="Pluginlib XML filename"),
):
    """List plugins of a pluginlib file with their descriptions, by package name and filename."""
    path = pathlib.Path(get_package_share_directory(package), filename)

    print_plugins(path=path)


@app.command()
def by_path(path: str = typer.Option(..., help="Pluginlib XML absolute file path")):
    """List plugins of a pluginlib file with their descriptions, by their path."""
    print_plugins(path=pathlib.Path(path))


@app.command()
def measurements():
    """List measurement plugins with their descriptions."""
    print_plugins(
        path=pathlib.Path(get_package_share_directory("dc_measurements"), "measurement_plugin.xml")
    )


@app.command()
def conditions():
    """List condition plugins with their descriptions."""
    print_plugins(
        path=pathlib.Path(get_package_share_directory("dc_conditions"), "condition_plugin.xml")
    )


@app.command()
def destinations():
    """List destination plugins with their descriptions."""
    print_plugins(
        path=pathlib.Path(get_package_share_directory("dc_destinations"), "destination_plugin.xml")
    )


if __name__ == "__main__":
    app()
