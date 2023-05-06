import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
import streamlit as st
from backend import PGSQLService
from config import GetDataMode, config
from lib import Section
from pages import Header, Sidebar
from plotly.subplots import make_subplots


class OS(Section):
    def __init__(self) -> None:
        st.subheader("OS")
        self.os = ""
        self.kernel = ""
        self.memory = ""
        self.cpu = ""
        self.load_data()
        self.display_data()

    @Section.handler_load_data_none
    def load_data(self) -> None:
        if config.BACKEND == config.BACKEND.POSTGRESQL:
            self.os, self.cpu, self.kernel, self.memory = PGSQLService().get_os(
                robot_name=st.session_state.robot_name
            )

    @Section.handler_display_data_none
    def display_data(self) -> None:
        assert all([self.os, self.kernel, self.memory, self.cpu])
        col1, col2, col3, col4 = st.columns(4)
        col1.metric("Operating System", self.os)
        col2.metric("Kernel", self.kernel)
        col3.metric("Cpus", self.cpu)
        col4.metric("Memory", f"{self.memory}Gb")


class Memory:
    def __init__(self) -> None:
        self.data = None
        self.df = None
        self.fig = None
        self.load_data()
        self.create_plotly_figure()
        self.display_data()

    def load_data(self) -> None:
        if st.session_state.mode == GetDataMode.RUN_ID_MODE:
            self.data = PGSQLService().get_memory(
                robot_name=st.session_state.robot_name, run_id=st.session_state.run_id
            )
            self.df = pd.DataFrame(self.data, columns=["Date", "Memory used"])

    def create_plotly_figure(self) -> None:
        self.fig = px.line(
            self.df,
            x="Date",
            y="Memory used",
            title="Memory used over time",
            markers=True,
        )
        self.fig.update_xaxes(
            # dtick=30 * 60 * 1000,
            showgrid=True,
            title_text=None,
        )
        self.fig.update_yaxes(
            range=[0, 100],
            tick0=0,
            dtick=10,
            showgrid=True,
            title_text=None,
        )

    def display_data(self) -> None:
        st.plotly_chart(self.fig, use_container_width=True)


class CPU:
    def __init__(self) -> None:
        self.data = None
        self.processes = None
        self.df = None
        self.fig = None
        self.load_data()
        self.create_plotly_figure()
        self.display_data()

    def load_data(self) -> None:
        if st.session_state.mode == GetDataMode.RUN_ID_MODE:
            self.data = PGSQLService().get_cpu_average(
                robot_name=st.session_state.robot_name, run_id=st.session_state.run_id
            )
            self.df = pd.DataFrame(self.data, columns=["Date", "CPU", "Processes"])

    def create_plotly_figure(self) -> None:
        # Create figure with secondary y-axis
        self.fig = make_subplots(specs=[[{"secondary_y": True}]])

        # Add traces
        self.fig.add_trace(
            go.Scatter(
                x=self.df["Date"],
                y=self.df["CPU"],
                name="CPU over time",
                mode="lines+markers",
            ),
            secondary_y=False,
        )
        self.fig.update_xaxes(
            range=[self.df["Date"].min(), self.df["Date"].max()],
            # dtick=30 * 60 * 1000,
            tick0=self.df["Date"].min(),
            showgrid=True,
        )
        self.fig.update_yaxes(
            range=[0, 100],
            tick0=0,
            dtick=10,
            secondary_y=False,
            title_text=None,
        )

        self.fig.add_trace(
            go.Scatter(
                x=self.df["Date"], y=self.df["Processes"], name="Processes", mode="lines+markers"
            ),
            secondary_y=True,
        )
        self.fig.update_layout(legend={"orientation": "h"}, title="CPU and processes over time")

    def display_data(self) -> None:
        st.plotly_chart(self.fig, use_container_width=True)


def main():
    st.set_page_config(page_title="ROS 2 Data Collection - System", layout="wide")

    st.title("System data")
    Header()
    Sidebar()
    OS()
    st.divider()
    Memory()
    CPU()


if __name__ == "__main__":
    main()
