import pandas as pd
import plotly.graph_objects as go
import streamlit as st
from backend import PGSQLService, minio_client
from config import Backend, Storage, config
from lib import Section
from pages import Header, Sidebar
from plotly.subplots import make_subplots


class Speed(Section):
    supported_backends = [config.BACKEND.POSTGRESQL]
    supported_storages = []

    def __init__(self, backend: Backend = config.BACKEND) -> None:
        super().__init__(backend=backend)
        st.subheader("Speed and command velocity over time")
        self.speed = None
        self.speed_df = None
        self.cmd_vel = None
        self.cmd_vel_df = None
        self.fig = None
        self.load_data()
        self.create_plotly_figure()
        self.display_data()

    @Section.handler_load_data_backend_not_implemented
    @Section.handler_load_data_none
    def load_data(self) -> None:
        if self.backend == config.BACKEND.POSTGRESQL:
            self.speed = PGSQLService.get_speed(
                robot_name=st.session_state.robot_name,
                run_id=st.session_state.get("run_id", ""),
                start_date=st.session_state.get("start_date", None),
                end_date=st.session_state.get("end_date", None),
            )
            self.speed_df = pd.DataFrame(self.speed, columns=["Date", "Speed"])
            self.cmd_vel = PGSQLService.get_cmd_vel(
                robot_name=st.session_state.robot_name,
                run_id=st.session_state.get("run_id", ""),
                start_date=st.session_state.get("start_date", None),
                end_date=st.session_state.get("end_date", None),
            )
            self.cmd_vel_df = pd.DataFrame(self.cmd_vel, columns=["Date", "Command velocity"])

    @Section.display_if_data_in_df("speed_df", "cmd_vel_df")
    def create_plotly_figure(self) -> None:
        if self.cmd_vel_df.empty or self.speed_df.empty:
            secondary_y = False
            unique_plot = True
        # Create figure with secondary y-axis
        else:
            secondary_y = True
            unique_plot = False
        # If no data, do not create figure
        self.fig = make_subplots(specs=[[{"secondary_y": secondary_y}]])
        self.fig.update_xaxes(
            range=[self.speed_df["Date"].min(), self.speed_df["Date"].max()],
            tick0=self.speed_df["Date"].min(),
            showgrid=True,
        )
        if not self.speed_df.empty:
            # Add traces
            self.fig.add_trace(
                go.Scatter(
                    x=self.speed_df["Date"],
                    y=self.speed_df["Speed"],
                    name="Speed",
                    mode="lines+markers",
                ),
                secondary_y=False,
            )
            self.fig.update_yaxes(
                tick0=0,
                dtick=0.1,
                secondary_y=False,
            )
            average_speed_run = PGSQLService.get_average_speed(
                robot_name=st.session_state.robot_name,
                run_id=st.session_state.get("run_id", ""),
                start_date=st.session_state.get("start_date", None),
                end_date=st.session_state.get("end_date", None),
            )
            if average_speed_run:
                speed_annotation_text = f"Avg speed ({round(average_speed_run,2)})"
            else:
                speed_annotation_text = 0
            self.fig.add_hline(
                y=average_speed_run,
                line_width=3,
                line_dash="dash",
                line_color="green",
                annotation_text=speed_annotation_text,
                annotation_position="top left",
                annotation_font={"color": "green", "size": 15},
            )

        if not self.cmd_vel_df.empty:
            # We need to add the y axis if it is a single plot
            self.fig.add_trace(
                go.Scatter(
                    x=self.cmd_vel_df["Date"],
                    y=self.cmd_vel_df["Command velocity"],
                    name="Command velocity",
                    mode="lines+markers",
                ),
                secondary_y=secondary_y,
            )
            self.fig.update_yaxes(
                tick0=0,
                dtick=0.1,
                secondary_y=secondary_y,
            )

        # Legend not shown for single plot
        # https://stackoverflow.com/a/61312761/1717026
        if unique_plot:
            self.fig["data"][0]["showlegend"] = True
        self.fig.update_layout(legend={"orientation": "h"}, title="")

    @Section.handler_display_data_backend_not_implemented
    @Section.handler_display_data_none
    def display_data(self) -> None:
        assert any([not self.cmd_vel_df.empty, not self.speed_df.empty])
        st.plotly_chart(self.fig, use_container_width=True)


class Robot(Section):
    supported_backends = [Backend.POSTGRESQL]
    supported_storages = []

    def __init__(self, backend: Backend = config.BACKEND) -> None:
        super().__init__(backend=backend)
        self.total_distance = -1
        self.average_speed = -1
        self.cols_data = []
        self.col = None
        self.load_data()
        self.display_data()

    @Section.handler_load_data_backend_not_implemented
    @Section.handler_load_data_none
    def load_data(self):
        if self.backend == Backend.POSTGRESQL:
            self.total_distance = PGSQLService.get_total_distance(
                robot_name=st.session_state.robot_name,
                run_id=st.session_state.get("run_id", ""),
                start_date=st.session_state.get("start_date", None),
                end_date=st.session_state.get("end_date", None),
            )
            self.average_speed = PGSQLService.get_average_speed(
                robot_name=st.session_state.robot_name,
                run_id=st.session_state.get("run_id", ""),
                start_date=st.session_state.get("start_date", None),
                end_date=st.session_state.get("end_date", None),
            )
            self.cols_data = [
                {
                    "title": "Average speed",
                    "prefix": "",
                    "suffix": "m/s",
                    "value": round(self.average_speed, 2),
                    "err_value": -1,
                },
                {
                    "title": "Distance traveled",
                    "prefix": "",
                    "suffix": "m",
                    "value": round(self.total_distance, 2),
                    "err_value": -1,
                },
            ]
            # Data we have first and unknown later
            self.cols_data = sorted(self.cols_data, key=lambda x: x["value"] != x["err_value"])[
                ::-1
            ]

    @Section.handler_display_data_none_cols
    def display_data(self):
        cols = st.columns(len(self.cols_data))
        for count, col in enumerate(cols):
            with col:
                self.col = col
                self.count = count
                assert self.cols_data[count]["value"] != self.cols_data[count]["err_value"]
                st.metric(
                    self.cols_data[count]["title"],
                    f"{self.cols_data[count]['prefix']}"
                    f"{self.cols_data[count]['value']}"
                    f"{self.cols_data[count]['suffix']}",
                )


class CameraImages(Section):
    supported_backends = [Backend.POSTGRESQL]
    supported_storages = [Storage.MINIO]

    def __init__(
        self, backend: Backend = config.BACKEND, storage: Storage = config.STORAGE
    ) -> None:
        super().__init__(backend=backend, storage=storage)
        st.subheader("Camera images")
        self.image_data = []
        self.df = None
        self.load_data()
        self.display_data()

    @Section.handler_load_data_backend_not_implemented
    @Section.handler_load_data_none
    def load_data(self) -> None:
        if self.backend == Backend.POSTGRESQL:
            self.image_data = PGSQLService.get_camera_images(
                robot_name=st.session_state.robot_name,
                run_id=st.session_state.get("run_id", ""),
                start_date=st.session_state.get("start_date", None),
                end_date=st.session_state.get("end_date", None),
                storage=config.STORAGE,
            )
            self.df = pd.DataFrame(
                self.image_data,
                columns=[
                    "Camera Name",
                    "Raw remote path",
                    "Rotated remote path",
                    "Inspected remote path",
                    "Run ID",
                    "Date",
                ],
            )

    @Section.handler_display_data_backend_not_implemented
    @Section.handler_display_data_storage_not_implemented
    @Section.handler_display_data_none
    @Section.display_if_data_in_df("df")
    def display_data(self) -> None:
        camera_names = self.df["Camera Name"].unique()
        if self.storage == Storage.MINIO:
            camera_tabs = st.tabs(camera_names)

            for camera_tab_index, camera_tab in enumerate(camera_tabs):
                with camera_tab:
                    images_camera = self.df[
                        self.df["Camera Name"] == camera_names[camera_tab_index]
                    ]
                    df_images_camera_raw = images_camera[images_camera["Raw remote path"].notna()]
                    df_images_camera_rotated = images_camera[
                        images_camera["Rotated remote path"].notna()
                    ]
                    df_images_camera_inspected = images_camera[
                        images_camera["Inspected remote path"].notna()
                    ]

                    raw_tab, rotated_tab, inspected_tab = st.tabs(["Raw", "Rotated", "Inspected"])

                    with raw_tab:
                        if self.storage == Storage.MINIO:
                            if len(df_images_camera_raw):
                                cols = raw_tab.columns(3)
                                for i in range(len(df_images_camera_raw)):
                                    cols[i % 3].image(
                                        image=minio_client.get_presigned_url(
                                            "GET",
                                            config.MINIO_BUCKET,
                                            df_images_camera_raw.loc[i, "Raw remote path"],
                                        ),
                                        caption=df_images_camera_raw.loc[i, "Date"],
                                    )
                            else:
                                st.info("No data")
                    with rotated_tab:
                        if self.storage == Storage.MINIO:
                            if len(df_images_camera_rotated):
                                cols = rotated_tab.columns(3)
                                for i in range(len(df_images_camera_rotated)):
                                    cols[i % 3].image(
                                        image=minio_client.get_presigned_url(
                                            "GET",
                                            config.MINIO_BUCKET,
                                            df_images_camera_rotated.loc[i, "Rotated remote path"],
                                        ),
                                        caption=df_images_camera_raw.loc[i, "Date"],
                                    )
                            else:
                                st.info("No data")
                    with inspected_tab:
                        if self.storage == Storage.MINIO:
                            cols = inspected_tab.columns(3)
                            if len(df_images_camera_inspected):
                                for i in range(len(df_images_camera_inspected)):
                                    cols[i % 3].image(
                                        image=minio_client.get_presigned_url(
                                            "GET",
                                            config.MINIO_BUCKET,
                                            df_images_camera_inspected.loc[
                                                i, "Rotated remote path"
                                            ],
                                        ),
                                        caption=df_images_camera_inspected.loc[i, "Date"],
                                    )
                            else:
                                st.info("No data")


def main():
    st.set_page_config(page_title="ROS 2 Data Collection - Robot", layout="wide")

    st.title("Robot data")
    Header()
    Sidebar()
    Robot()
    st.divider()
    Speed()
    CameraImages()


if __name__ == "__main__":
    main()
