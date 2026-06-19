import os
import typing
from glob import glob

from setuptools import find_packages, setup

package_name = "mongodb_store"


def glob_files(
    directory: str,
    file_matcher: str = "*",
    recursive=True,
) -> typing.Tuple[str, typing.List[str]]:
    """
    Glob files in the given directory to use in the data files part of setup.

    Args:
        directory: Directory to glob
        file_matcher: Shell-style matching string used to match files to glob
        recursive: Recurse over subdirectories. This probably doesn't work because subdirectories also get globbed
                   and setup doesn't like that.

    Returns:
        Tuple of the directory in the share location, and list of globbed files
    """
    return os.path.join("share", package_name, directory), glob(
        os.path.join(directory, "" if not recursive else "**", file_matcher),
        recursive=recursive,
    )


setup(
    name=package_name,
    version="2.0.3",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        glob_files("launch", "*launch.[pxy][yma]*"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michal Staniaszek",
    maintainer_email="michal@robots.ox.ac.uk",
    description="MongoDB interaction for ROS2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "config_manager = mongodb_store.scripts.config_manager:main",
            "example_message_store_client = mongodb_store.scripts.example_message_store_client:main",
            "example_multi_event_log = mongodb_store.scripts.example_multi_event_log:main",
            "message_store_node = mongodb_store.message_store_node:main",
            "mongo_bridge = mongodb_store.scripts.mongo_bridge:main",
            "mongodb_play = mongodb_store.scripts.mongodb_play:main",
            "mongodb_server = mongodb_store.mongodb_server:main",
            "replicator_client= mongodb_store.scripts.replicator_client:main",
            "replicator_node = mongodb_store.scripts.replicator_node:main",
        ],
    },
)
