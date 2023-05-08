import pathlib

import streamlit as st


def set_congrats_text():
    current_file_path = pathlib.Path(__file__).parent.resolve()
    with open(f"{current_file_path}/../static/html/congrats.html") as f:
        st.markdown(f.read(), unsafe_allow_html=True)


def main():
    st.set_page_config(page_title="ROS 2 Data Collection - End", layout="wide")
    st.title("Thanks!")
    set_congrats_text()


if __name__ == "__main__":
    main()
