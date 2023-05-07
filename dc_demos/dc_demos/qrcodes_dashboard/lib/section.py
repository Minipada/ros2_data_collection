import logging
from abc import ABCMeta, abstractmethod

import streamlit as st
from config import Backend, config


class Section(metaclass=ABCMeta):
    def __init__(self, backend: Backend = config.BACKEND) -> None:
        self.backend = backend
        self.supported_backends = []

    @classmethod
    def handler_load_data_not_implemented(cls, func):
        def wrapper(self, *args, **kwargs):
            if self.backend in self.supported_backends:
                result = func(self, *args, **kwargs)
                return result
            else:
                logging.warning("Backend not supported for this function")

        return wrapper

    @staticmethod
    def handler_load_data_none(func):
        def wrapper(self):
            try:
                func(self)
            except (TypeError, AssertionError):
                logging.info("No data")

        return wrapper

    @staticmethod
    def handler_display_data_none(func):
        def wrapper(self):
            try:
                func(self)
            except AssertionError:
                st.info("No data")

        return wrapper

    @classmethod
    def handler_display_data_not_implemented(cls, func):
        def wrapper(self, *args, **kwargs):
            if self.backend in self.supported_backends:
                result = func(self, *args, **kwargs)
                return result
            else:
                st.warning("Backend not supported for this function", icon="⚠️")

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
                    st.info("No data")

        return wrapper

    @staticmethod
    def draw_figure_if_data(func):
        def wrapper(self):
            if self.df is not None and not self.df.empty:
                func(self)

        return wrapper

    @abstractmethod
    def load_data(self) -> None:
        pass

    @abstractmethod
    def display_data(self) -> None:
        pass
