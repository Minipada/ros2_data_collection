import pandas as pd
import plotly.express as px
import streamlit as st
from backend import PGSQLService
from config import Backend, config
from lib import Section
from pages import Header, Sidebar


class TCPServerHealth(Section):
    supported_backends = [Backend.POSTGRESQL]
    supported_storages = []

    def __init__(self, backend: Backend = config.BACKEND) -> None:
        super().__init__(backend=backend)
        st.subheader("Infrastructure")
        self.data = None
        self.df = None
        self.fig = []
        self.load_data()
        self.create_plotly_figures()
        self.display_data()

    @Section.handler_load_data_backend_not_implemented
    @Section.handler_load_data_none
    def load_data(self) -> None:
        if self.backend == Backend.POSTGRESQL:
            self.data = PGSQLService.get_tcp_health(
                robot_name=st.session_state.robot_name,
                run_id=st.session_state.get("run_id", ""),
                start_date=st.session_state.get("start_date", None),
                end_date=st.session_state.get("end_date", None),
            )
            self.df = pd.DataFrame(
                self.data, columns=["Server Name", "Host", "Port", "Active", "Date"]
            )

    @Section.display_if_data_in_df("df")
    def create_plotly_figures(self) -> None:
        fig = None
        for server_name in self.df["Server Name"].unique():
            fig = px.line(
                self.df[self.df["Server Name"] == server_name],
                x="Date",
                y="Active",
                title=f"{server_name} health",
                markers=True,
            )
            fig.update_xaxes(
                showgrid=True,
                title_text=None,
            )
            fig.update_yaxes(
                range=[True, False],
                showgrid=True,
                title_text=None,
            )
            self.fig.append(fig)

    @Section.handler_display_data_backend_not_implemented
    @Section.handler_display_data_none
    def display_data(self) -> None:
        assert self.df.empty is False
        for fig in self.fig:
            st.plotly_chart(fig, use_container_width=True)


def main():
    st.set_page_config(page_title="ROS 2 Data Collection - Infrastructure", layout="wide")

    st.title("Infrastructure data")
    Header()
    Sidebar()
    TCPServerHealth()


if __name__ == "__main__":
    main()
