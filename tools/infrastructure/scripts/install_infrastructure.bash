#!/bin/bash
set -e

# Arguments parsing
help() {
usage="$(basename "$0") --tool=<TOOL> --install-type=<INSTALL_TYPE>

Install infrastructure tool on the system.

Arguments:
--help/-h           Show this help text
--tool              Set the tool to install [chromium, influxdb, grafana, minio, postgresql] (mandatory)
--install-type      Set the type of installation [docker, native] (mandatory)
"
    echo "${usage}"
}

for i in "$@"
do
case $i in
    --tool=*)
    TOOL="${i#*=}"
    shift # past argument=value
    ;;
    --install-type=*)
    INSTALL_TYPE="${i#*=}"
    shift # past argument=value
    ;;
    --help)
    HELP=YES
    shift # past argument with no value
    ;;
    *)
          # unknown option
    ;;
esac
done

# Arguments check
if [ -n "${HELP}" ]; then
    help
    exit 0
fi

if [ -z "${TOOL}" ]; then
    echo "You need to set the tool to install"
    help
    exit 1
fi

if [ -z "${INSTALL_TYPE}" ]; then
    echo "You need to set the type of installation (docker, native)"
    help
    exit 1
fi

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
SCRIPTDIR="$(dirname "${SCRIPTPATH}")"

ID_LIKE="$(awk -F= '$1=="ID_LIKE" { print $2 ;}' /etc/os-release)"

# Docker installation
function debian_docker (){
    if ! command -v docker &> /dev/null; then
        echo "Docker is not installed. Installing..."
        if ! command -v curl &> /dev/null; then
            echo "Curl is not installed. Installing..."
            sudo apt update -qq
            sudo apt install -y curl
        fi
        curl -fsSL https://get.docker.com -o get-docker.sh
    fi
    echo "Docker downloaded."
}

function arch_docker (){
    if ! command -v docker &> /dev/null; then
        echo "Docker is not installed. Installing..."
        if ! command -v curl &> /dev/null; then
            echo "Curl is not installed. Installing..."
            sudo pacman -Syy
            sudo pacman -S curl --noconfirm
        fi
        curl -fsSL https://get.docker.com -o get-docker.sh
    fi
    echo "Docker downloaded."
}

function init_docker() {
    sh get-docker.sh
    rm get-docker.sh
    sudo usermod -aG docker "$(whomai)"
    echo "Docker installed."
    echo "Please logout to use docker without root permissions!"
}

# Native init
function debian_native (){
    sudo apt update -qq
}

function arch_native (){
    sudo pacman -Syy
}

function debian_native_postgresql(){
    sudo apt install postgresql postgresql-contrib -y
    if [[ $(ps -p 1 -o comm=) == "systemd" ]]; then
        sudo systemctl daemon-reload
        sudo systemctl enable postgresql
        sudo systemctl start postgresql
    else
        sudo service postgresql enable
        sudo service postgresql start
    fi
    sudo -i -u postgres createuser dc
    sudo -i -u postgres createdb dc
    sudo -i -u postgres psql -c "alter user dc with password 'password'"

    # Install apache
    sudo apt install apache2 -y

    echo "PostgreSQL installed."
    echo "Username: dc"
    echo "Password: password"
    echo "Port: 5432"
}

function debian_native_adminer(){
    sudo apt install adminer -y
    sudo ln -s /etc/apache2/conf-available/adminer.conf /etc/apache2/conf-enabled/

    if [[ $(ps -p 1 -o comm=) == "systemd" ]]; then
        sudo systemctl daemon-reload
        sudo systemctl enable apache2
        sudo systemctl start apache2
    else
        sudo service apache2 enable
        sudo service apache2 start
    fi

    echo "Adminer installed."
    echo "Open your browser at http://localhost/adminer"
    echo "Port: 80"
}

function debian_native_grafana(){
    sudo apt-get install -y apt-transport-https software-properties-common wget
    sudo wget -q -O /usr/share/keyrings/grafana.key https://apt.grafana.com/gpg.key

    echo "deb [signed-by=/usr/share/keyrings/grafana.key] https://apt.grafana.com stable main" | sudo tee -a /etc/apt/sources.list.d/grafana.list

    # Update the list of available packages
    sudo apt-get update -qq

    # Install the latest OSS release:
    sudo apt-get install grafana -y

    if [[ $(ps -p 1 -o comm=) == "systemd" ]]; then
        sudo systemctl daemon-reload
        sudo systemctl enable grafana-server
        sudo systemctl start grafana-server
    else
        sudo service grafana-server enable
        sudo service grafana-server start
    fi

    echo "Grafana installed."
    echo "Open your browser at http://localhost:3000"
}

function debian_native_chromium(){
    sudo apt install chromium-browser

    echo "Chromium installed."
}

function debian_native_influxdb(){
    sudo apt install -y wget
    wget -q https://repos.influxdata.com/influxdata-archive_compat.key
    # shellcheck disable=SC2002
    echo '393e8779c89ac8d958f81f942f9ad7fb82a25e133faddaf92e15b16e6ac9ce4c influxdata-archive_compat.key' | sha256sum -c && cat influxdata-archive_compat.key | $ gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/influxdata-archive_compat.gpg > /dev/null
    echo 'deb [signed-by=/etc/apt/trusted.gpg.d/influxdata-archive_compat.gpg] https://repos.influxdata.com/debian stable main' | sudo tee /etc/apt/sources.list.d/influxdata.list
    sudo apt-get update -qq && sudo apt-get install influxdb

    if [[ $(ps -p 1 -o comm=) == "systemd" ]]; then
        sudo systemctl daemon-reload
        sudo systemctl unmask influxdb.service
        sudo systemctl enable influxdb
        sudo systemctl start influxdb
    else
        sudo service influxdb enable
        sudo service influxdb start
    fi

    echo "Influxdb installed."
    echo "User: admin"
    echo "Password: admin"
    echo "Database: dc"
}

function debian_native_minio(){
    sudo apt install wget -y
    wget -O minio https://dl.minio.io/server/minio/release/linux-amd64/minio
    chmod +x minio
    sudo mv minio /usr/local/bin/
    sudo groupadd -f -r minio-user
    sudo useradd -M -r -g minio-user minio-user || true
    sudo cp "${SCRIPTDIR}/scripts/minio/minio.conf" /etc/default/minio
    sudo cp "${SCRIPTDIR}/scripts/minio/minio.service" /etc/systemd/system/

    if [[ $(ps -p 1 -o comm=) == "systemd" ]]; then
        sudo systemctl daemon-reload
        sudo systemctl enable minio.service
        sudo systemctl start minio.service
    else
        sudo service minio enable
        sudo service minio start
    fi

    echo "MinIO installed."
    echo "Open your browser at http://localhost:9001"
    echo "User: minioadmin"
    echo "Password: minioadmin"
}

function start_container() {
    cd "${SCRIPTDIR}/"

    DOCKER_COMPOSE_PATH="docker/docker-compose.${TOOL}.yaml"

    if [ -f "${DOCKER_COMPOSE_PATH}" ]; then
        docker compose -f "docker/docker-compose.${TOOL}.yaml" up -d
        echo "Installation done!"
        exit 0
    else
        echo "Tool docker compose not available. Exiting."
        exit 1
    fi
}

# Install and start tool if possible
if [ "${INSTALL_TYPE}" == "docker" ]; then
    if ! command -v docker &> /dev/null; then
        "${ID_LIKE}_docker"
        init_docker
    else
        start_container
    fi
else
    # shellcheck disable=SC2046
    if [ $(type -t "${ID_LIKE}_${INSTALL_TYPE}") == "function" ]; then
        eval "${ID_LIKE}_${INSTALL_TYPE}"
    else
        echo "Install type not available. Exiting."
        exit 1
    fi

    # shellcheck disable=SC2046
    if [ $(type -t "${ID_LIKE}_${INSTALL_TYPE}_${TOOL}") == "function" ]; then
        eval "${ID_LIKE}_${INSTALL_TYPE}_${TOOL}"
    else
        echo "Tool not available. Exiting."
        exit 1
    fi
fi
