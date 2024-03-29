"""Define common header for each page"""

import datetime
import pathlib

import streamlit as st
from backend import PGSQLService
from config import GetDataMode


class Header:
    def __init__(self) -> None:
        self.set_css()
        self.start_dom = None
        self.end_dom = None

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
        if st.session_state.mode == GetDataMode.RUN_ID:
            self.run_ids = PGSQLService.get_unique_run_ids(robot_name=st.session_state.robot_name)
            st.selectbox(
                "Select a run id",
                options=[robot_name[1] for robot_name in self.run_ids],
                key="run_id",
                format_func=self.set_format_selectbox,
            )
            st.session_state.start_date = ""
            st.session_state.end_date = ""
        else:
            # if st.session_state.mode == GetDataMode.TIME_MODE:
            now: datetime.datetime = datetime.datetime.now()
            if st.session_state.mode == GetDataMode.TODAY:
                st.session_state.start_date = now.replace(hour=0, minute=0, second=0, microsecond=0)
                st.session_state.end_date = now
            elif st.session_state.mode == GetDataMode.YESTERDAY:
                yesterday: datetime.datetime = now - datetime.timedelta(days=1)
                st.session_state.start_date = datetime.datetime(
                    yesterday.year, yesterday.month, yesterday.day, 0, 0, 0
                )
                st.session_state.end_date = datetime.datetime(
                    yesterday.year, yesterday.month, yesterday.day, 23, 59, 59
                )
            elif st.session_state.mode == GetDataMode.THIS_WEEK:
                monday = now.date() - datetime.timedelta(days=now.weekday())
                st.session_state.start_date = datetime.datetime(
                    monday.year, monday.month, monday.day, 0, 0, 0
                )
                st.session_state.end_date = st.session_state.start_date + datetime.timedelta(
                    days=6, hours=23, minutes=59, seconds=59
                )
            elif st.session_state.mode == GetDataMode.LAST_WEEK:
                last_monday = now.date() - datetime.timedelta(days=now.weekday() + 7)
                st.session_state.start_date = datetime.datetime(
                    last_monday.year, last_monday.month, last_monday.day, 0, 0, 0
                )
                st.session_state.end_date = st.session_state.start_date + datetime.timedelta(
                    days=6, hours=23, minutes=59, seconds=59
                )
            elif st.session_state.mode == GetDataMode.THIS_MONTH:
                start_of_month = datetime.datetime(now.year, now.month, 1, 0, 0, 0)
                # We get the last day of the month by adding 4 days to the 28th day
                # of the current month, which handles edge cases like leap years.
                last_day_of_month = datetime.date(now.year, now.month, 28) + datetime.timedelta(
                    days=4
                )
                st.session_state.start_date = start_of_month
                st.session_state.end_date = datetime.datetime.combine(
                    last_day_of_month, datetime.datetime.max.time()
                )
            elif st.session_state.mode == GetDataMode.LAST_3_MONTHS:
                # Calculate the start of the month 3 months ago
                start_of_month_3_months_ago = datetime.date(
                    now.year, now.month, 1
                ) - datetime.timedelta(days=7 * 4 * 3)
                # Calculate the start and end of the previous 3 months
                start_of_3_months_ago = datetime.datetime(
                    start_of_month_3_months_ago.year, start_of_month_3_months_ago.month, 1, 0, 0, 0
                )

                st.session_state.start_date = start_of_3_months_ago
                st.session_state.end_date = now
            elif st.session_state.mode == GetDataMode.SELECT_TIME:
                result = PGSQLService.get_start_end_date(robot_name=st.session_state.robot_name)
                if (result[1] - result[0]) > datetime.timedelta(days=1):
                    step = datetime.timedelta(days=1)
                elif (result[1] - result[0]) > datetime.timedelta(hours=1):
                    step = datetime.timedelta(minutes=30)
                else:
                    step = datetime.timedelta(minutes=1)

                slider = st.slider(
                    label="Time range",
                    min_value=result[0],
                    max_value=result[1],
                    value=(result[0], result[1]),
                    format="YYYY-MM-DD HH:mm",
                    key="date_range",
                    on_change=self.set_dates,
                    step=step,
                )
                st.session_state.start_date = slider[0]
                st.session_state.end_date = slider[1]
            st.session_state.pop("run_id", None)

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
            "Data filter",
            [x.value for x in GetDataMode],
            key="mode",
            # Keep the state when switching pages
            # Defaults to the first value
            index=[x.value for x in GetDataMode].index(
                st.session_state.get("mode", list(GetDataMode)[0].value)
            ),
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
