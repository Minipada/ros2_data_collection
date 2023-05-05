import pandas as pd
import plotly.express as px
import streamlit as st
from backend import PGSQLService
from pages import Header, Sidebar


class TCPServerHealth:
    def __init__(self) -> None:
        self.data = None
        self.fig = []
        self.load_data()
        self.create_plotly_figures()
        self.display_data()

    def load_data(self) -> None:
        self.data = PGSQLService().get_tcp_health(
            robot_name=st.session_state.robot_name, run_id=st.session_state.run_id
        )
        self.df = pd.DataFrame(self.data, columns=["Server Name", "Host", "Port", "Active", "Date"])

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

    def display_data(self) -> None:
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
