# ghcr.io/minipada/ros2_data_collection:${ROS_DISTRO}-doc
# Image to build the documentation
FROM rust:1.68.2-bullseye AS ros2_dc_doc_builder

WORKDIR /

COPY requirements.txt .
COPY requirements-dev.txt .

RUN apt-get -q update && \
    # Python3
    apt-get -q install --no-install-recommends -y python-is-python3 python3-pip && \
    pip3 install --no-cache-dir -r requirements.txt -r requirements-dev.txt && \
    cargo install mdbook mdbook-admonish mdbook-linkcheck mdbook-mermaid mdbook-open-on-gh && \
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*
