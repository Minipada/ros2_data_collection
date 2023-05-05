import os


def get_tz() -> str:
    """Get local timezone from localtime information."""
    return "/".join(os.path.realpath("/etc/localtime").split("/")[-2:])
