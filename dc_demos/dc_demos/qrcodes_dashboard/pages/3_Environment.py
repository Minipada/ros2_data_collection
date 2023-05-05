import pathlib

import streamlit as st
from backend import PGSQLService, minio_client
from config import Storage, config
from pages import Header, Sidebar


class Map:
    def __init__(self) -> None:
        self.last_map_url_png = None
        self.base_file_name = None
        self.last_map_png = None
        self.last_map_pgm = None
        self.last_map_yaml = None
        self.load_data()
        if self.last_map_paths:
            self.display_map()

    def load_data(self):
        self.last_map_paths = PGSQLService().get_last_map(
            robot_name=st.session_state.robot_name,
            storage=config.STORAGE,
        )
        if not self.last_map_paths:
            return

        if config.STORAGE == Storage.MINIO:
            self.last_map_url_png = minio_client.get_presigned_url(
                "GET",
                config.MINIO_BUCKET,
                self.last_map_paths[0],
            )
        self.base_file_name = pathlib.Path(
            f"{st.session_state.robot_name}_"
            f"{pathlib.Path(self.last_map_url_png.split('/')[-1]).with_suffix('')}"
        )

        if config.STORAGE == Storage.MINIO:
            self.last_map_png = minio_client.get_object(
                config.MINIO_BUCKET,
                self.last_map_paths[0],
            )
            self.last_map_pgm = minio_client.get_object(
                config.MINIO_BUCKET,
                self.last_map_paths[1],
            )
            self.last_map_yaml = minio_client.get_object(
                config.MINIO_BUCKET,
                self.last_map_paths[2],
            )

    def display_map(self):
        st.header("Map")
        col1, col2, col3, _, _, _ = st.columns(6)

        col1.download_button(
            label="Download PNG",
            data=self.last_map_png.read(),
            file_name=self.base_file_name.with_suffix(".png").as_posix(),
        )
        col2.download_button(
            label="Download PGM",
            data=self.last_map_pgm.read(),
            file_name=self.base_file_name.with_suffix(".pgm").as_posix(),
        )
        col3.download_button(
            label="Download YAML",
            data=self.last_map_yaml.read(),
            file_name=self.base_file_name.with_suffix(".yaml").as_posix(),
        )
        if self.last_map_url_png:
            st.image(self.last_map_url_png)


def main():
    st.set_page_config(page_title="ROS 2 Data Collection - Robot", layout="wide")

    st.title("Environment data")
    Header()
    Sidebar()
    Map()


if __name__ == "__main__":
    main()
