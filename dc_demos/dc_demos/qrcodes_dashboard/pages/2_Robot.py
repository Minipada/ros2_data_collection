import pandas as pd
import plotly.graph_objects as go
import streamlit as st
from backend import PGSQLService, minio_client
from config import GetDataMode, Storage, config
from pages import Header, Sidebar
from plotly.subplots import make_subplots


class Speed:
    def __init__(self) -> None:
        self.speed = None
        self.speed_df = None
        self.cmd_vel = None
        self.fig = None
        self.load_data()
        if not self.cmd_vel_df.empty or not self.speed_df.empty:
            self.create_plotly_figure()
            self.display_data()

    def load_data(self) -> None:
        if st.session_state.mode == GetDataMode.RUN_ID_MODE:
            self.speed = PGSQLService().get_speed(
                robot_name=st.session_state.robot_name, run_id=st.session_state.run_id
            )
            self.speed_df = pd.DataFrame(self.speed, columns=["Date", "Speed"])
            self.cmd_vel = PGSQLService().get_cmd_vel(
                robot_name=st.session_state.robot_name, run_id=st.session_state.run_id
            )
            self.cmd_vel_df = pd.DataFrame(self.cmd_vel, columns=["Date", "Command velocity"])

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
            average_speed_run = PGSQLService().get_average_speed(
                robot_name=st.session_state.robot_name, run_id=st.session_state.run_id
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
        self.fig.update_layout(
            legend={"orientation": "h"}, title="Speed and command velocity over time"
        )

    def display_data(self) -> None:
        if self.fig:
            st.plotly_chart(self.fig, use_container_width=True)


class Robot:
    def __init__(self) -> None:
        self.total_distance = 0
        self.load_data()
        self.display_metrics()

    def load_data(self):
        self.total_distance = PGSQLService().get_total_distance(
            robot_name=st.session_state.robot_name
        )
        self.average_speed = PGSQLService().get_average_speed(
            robot_name=st.session_state.robot_name
        )

    def display_metrics(self):
        col1, col2, _ = st.columns(3)
        col1.metric("Total distance traveled", f"{round(self.total_distance, 2)}m")
        col2.metric("Average speed", f"{round(self.average_speed, 2)}m/s")


class CameraImages:
    def __init__(self) -> None:
        self.load_data()
        if self.image_data:
            self.display_data()

    def load_data(self) -> None:
        self.image_data = PGSQLService().get_camera_images(
            robot_name=st.session_state.robot_name,
            run_id=st.session_state.run_id,
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

    def display_data(self) -> None:
        # Create a tab for each camera
        camera_names = self.df["Camera Name"].unique()
        if not camera_names.any():
            return
        camera_tabs = st.tabs(camera_names)

        for camera_tab_index, camera_tab in enumerate(camera_tabs):
            with camera_tab:
                images_camera = self.df[self.df["Camera Name"] == camera_names[camera_tab_index]]
                df_images_camera_raw = images_camera[images_camera["Raw remote path"].notna()]
                df_images_camera_rotated = images_camera[
                    images_camera["Rotated remote path"].notna()
                ]
                df_images_camera_inspected = images_camera[
                    images_camera["Inspected remote path"].notna()
                ]

                raw_tab, rotated_tab, inspected_tab = st.tabs(["Raw", "Rotated", "Inspected"])

                with raw_tab:
                    if config.STORAGE == Storage.MINIO:
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
                with rotated_tab:
                    if config.STORAGE == Storage.MINIO:
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
                with inspected_tab:
                    if config.STORAGE == Storage.MINIO:
                        cols = inspected_tab.columns(3)
                        for i in range(len(df_images_camera_inspected)):
                            cols[i % 3].image(
                                image=minio_client.get_presigned_url(
                                    "GET",
                                    config.MINIO_BUCKET,
                                    df_images_camera_inspected.loc[i, "Rotated remote path"],
                                ),
                                caption=df_images_camera_inspected.loc[i, "Date"],
                            )


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
