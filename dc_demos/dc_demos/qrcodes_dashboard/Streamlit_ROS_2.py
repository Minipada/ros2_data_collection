import pathlib

import streamlit as st
from pages import Header, Sidebar


def set_intro_text():
    current_file_path = pathlib.Path(__file__).parent.resolve()
    with open(f"{current_file_path}/static/html/welcome.html") as f:
        st.markdown(f.read(), unsafe_allow_html=True)


def main():
    st.set_page_config(page_title="ROS 2 Data Collection - Home", layout="wide")
    st.title("ROS 2 Data collection")
    Header()
    Sidebar()
    set_intro_text()


if __name__ == "__main__":
    main()
