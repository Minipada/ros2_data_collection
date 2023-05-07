import logging
from abc import ABCMeta, abstractmethod

import streamlit as st


class Section(metaclass=ABCMeta):
    @staticmethod
    def handler_load_data_none(func):
        def wrapper(self):
            try:
                func(self)
            except (TypeError, AssertionError):
                logging.warning("No data")

        return wrapper

    @staticmethod
    def handler_display_data_none(func):
        def wrapper(self):
            try:
                func(self)
            except AssertionError:
                st.warning("No data", icon="⚠️")

        return wrapper

    @staticmethod
    def handler_display_data_none_cols(func):
        def wrapper(self):
            try:
                func(self)
            except AssertionError:
                with self.col:
                    st.metric(
                        self.cols_data[self.count]["title"],
                        "",
                    )
                    st.warning("No data")

        return wrapper

    @handler_load_data_none
    @abstractmethod
    def load_data(self) -> None:
        pass

    @handler_display_data_none
    @abstractmethod
    def display_data(self) -> None:
        pass
