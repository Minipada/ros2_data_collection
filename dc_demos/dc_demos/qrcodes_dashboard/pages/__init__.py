"""Define common header for each page"""

import datetime
import pathlib

import streamlit as st
from backend import PGSQLService
from config import GetDataMode


class Header:
    def __init__(self) -> None:
        self.set_css()

    def set_css(self) -> None:
        """Set CSS used by the whole application."""
        current_file_path = pathlib.Path(__file__).parent.resolve()
        with open(f"{current_file_path}/../static/css/lineicons.css") as f:
            st.markdown(f"<style>{f.read()}</style>", unsafe_allow_html=True)


class Sidebar:
    def __init__(self) -> None:
        self.run_ids = []
        self.set_sidebar()

    def format_ts(self, ts: datetime.datetime) -> str:
        """Format timestamp for the Run ID selection.

        Args:
            ts (datetime.datetime): Timestamp from the database

        Returns:
            str: Formatted timestamp
        """
        return ts.strftime("%d %b %Y %H:%M:%S")

    def set_format_selectbox(self, run_id: str) -> str:
        """Format RUN id selectbox by including time when it started and ended.

        Args:
            run_id (str): Run ID

        Returns:
            str: Formatted string
        """
        start_time = [x[2] for x in self.run_ids if x[1] == run_id][0]
        end_time = [x[3] for x in self.run_ids if x[1] == run_id][0]
        return f"{run_id} ({self.format_ts(start_time)} -> {self.format_ts(end_time)})"

    def set_dates(self) -> None:
        st.session_state.start_date = st.session_state.date_range[0]
        st.session_state.end_date = st.session_state.date_range[1]

    def time_run_id_mode(self) -> None:
        """Show selectbox when Run ID mode selected and time range when time mode selected."""
        if st.session_state.mode == GetDataMode.RUN_ID_MODE:
            self.run_ids = PGSQLService.get_unique_run_ids(robot_name=st.session_state.robot_name)
            st.selectbox(
                "Select a run id",
                [robot_name[1] for robot_name in self.run_ids],
                key="run_id",
                format_func=self.set_format_selectbox,
            )
            st.session_state.start_date = ""
            st.session_state.end_date = ""

        if st.session_state.mode == GetDataMode.TIME_MODE:
            result = PGSQLService.get_start_end_date(robot_name=st.session_state.robot_name)
            st.slider(
                label="Time range",
                min_value=result[0],
                max_value=result[1],
                value=(result[0], result[1]),
                format="YYYY-MM-DD HH:mm:ss",
                key="date_range",
                on_change=self.set_dates,
            )
            st.session_state.run_id = ""

    def select_robot_mode_columns(self) -> None:
        """Create selectbox for Run ID and radio button to select mode."""
        result = PGSQLService.get_unique_robots()
        col1, col2 = st.columns(2)
        col1.selectbox(
            label="Select a robot",
            options=[robot_name[0] for robot_name in result],
            key="robot_name",
        )
        col2.radio(
            f"{GetDataMode.TIME_MODE.value} or {GetDataMode.RUN_ID_MODE}",
            (GetDataMode.RUN_ID_MODE.value, GetDataMode.TIME_MODE.value),
            key="mode",
        )
        self.time_run_id_mode()

    def set_sidebar(self) -> None:
        """Create sidebar content."""
        st.markdown(
            """
<style>
section[data-testid="stSidebar"] {
    width: 500px !important;
}
</style>
""",
            unsafe_allow_html=True,
        )
        with st.sidebar:
            self.select_robot_mode_columns()
            st.divider()
            current_file_path = pathlib.Path(__file__).parent.resolve()
            with open(f"{current_file_path}/../static/html/sidebar.html") as f:
                st.markdown(f.read(), unsafe_allow_html=True)
