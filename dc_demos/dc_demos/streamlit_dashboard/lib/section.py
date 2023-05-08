import logging
from abc import ABCMeta, abstractmethod

import streamlit as st
from config import Backend, Storage, config


class Section(metaclass=ABCMeta):
    def __init__(
        self, backend: Backend = config.BACKEND, storage: Storage = config.STORAGE
    ) -> None:
        self.backend = backend
        self.storage = storage

    @property
    @abstractmethod
    def supported_backends(self):
        pass

    @property
    @abstractmethod
    def supported_storages(self):
        pass

    @classmethod
    def handler_load_data_backend_not_implemented(cls, func):
        def wrapper(self, *args, **kwargs):
            if self.backend in self.supported_backends:
                result = func(self, *args, **kwargs)
                return result
            else:
                logging.warning("Backend not supported for this function")

        return wrapper

    @classmethod
    def handler_load_data_storage_not_implemented(cls, func):
        def wrapper(self, *args, **kwargs):
            if self.storage in self.supported_storages:
                result = func(self, *args, **kwargs)
                return result
            else:
                logging.warning("Storage not supported for this function")

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
    def handler_display_data_backend_not_implemented(cls, func):
        def wrapper(self, *args, **kwargs):
            if self.backend in self.supported_backends:
                result = func(self, *args, **kwargs)
                return result
            else:
                st.warning("Backend not supported for this function", icon="⚠️")

        return wrapper

    @classmethod
    def handler_display_data_storage_not_implemented(cls, func):
        def wrapper(self, *args, **kwargs):
            if self.storage in self.supported_storages:
                result = func(self, *args, **kwargs)
                return result
            else:
                st.warning("Storage not supported for this function", icon="⚠️")

        return wrapper

    @staticmethod
    def display_if_data_in_df(*var_names: str):
        def decorator(func):
            def wrapper(self):
                try:
                    if any(
                        getattr(self, name) is not None and not getattr(self, name).empty
                        for name in var_names
                    ):
                        func(self)
                except AttributeError:
                    st.error("Dataframe does not exist")

            return wrapper

        return decorator

    @abstractmethod
    def load_data(self) -> None:
        pass

    @abstractmethod
    def display_data(self) -> None:
        pass
