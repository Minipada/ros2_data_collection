import logging
from abc import ABCMeta, abstractmethod

import streamlit as st


class Section(metaclass=ABCMeta):
    @staticmethod
    def handler_load_data_none(func):
        def wrapper(self):
            try:
                return func(self)
            except TypeError:
                logging.warning("No data")

        return wrapper

    @staticmethod
    def handler_display_data_none(func):
        def wrapper(self):
            try:
                return func(self)
            except AssertionError:
                st.warning("No data", icon="⚠️")

        return wrapper

    @handler_load_data_none
    @abstractmethod
    def load_data(self) -> None:
        pass

    @handler_display_data_none
    @abstractmethod
    def display_data(self) -> None:
        pass
