# ghcr.io/minipada/ros2_data_collection:${ROS_DISTRO}-doc
# Image to build the documentation
FROM rust:1.68.2-bullseye AS ros2_dc_doc_builder

RUN cargo install mdbook mdbook-admonish mdbook-linkcheck mdbook-mermaid mdbook-open-on-gh
