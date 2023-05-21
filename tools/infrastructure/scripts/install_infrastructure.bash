#!/bin/bash
set -e

# Arguments parsing
help() {
usage="$(basename "$0") --tool=<TOOL> --install-type=<INSTALL_TYPE>

Install infrastructure tool on the system.

Arguments:
--help/-h           Show this help text
--tool              Set the tool to install [cromium, minio, pgsql] (mandatory)
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
            apt update -qq
            apt install -y curl
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

# Native PostgreSQL on Debian
function debian_native_pgsql(){
    # Install pgsql
    sudo apt install postgresql postgresql-contrib -y
    sudo service postgresql start
    sudo -i -u postgres createuser dc
    sudo -i -u postgres createdb dc
    sudo -i -u postgres psql -c "alter user dc with password 'password'"

    # Install apache
    sudo apt install apache2 -y

    # Install adminer
    sudo apt install adminer -y
    sudo ln -s /etc/apache2/conf-available/adminer.conf /etc/apache2/conf-enabled/

    sudo service apache2 restart

    echo "PostgreSQL and Adminer installed."
    echo "Open your browser at http://localhost/adminer"
    echo "Username: dc"
    echo "Password: password"
}

function debian_native_chromium(){
    sudo apt install software-properties-common -y
    sudo add-apt-repository ppa:saiarcot895/chromium-beta -y
    sudo apt install chromium-browser -y

    echo "Chromium installed."
}

function debian_native_minio(){
    sudo apt install wget -y
    # wget -O minio https://dl.minio.io/server/minio/release/linux-amd64/minio
    chmod +x minio
    sudo mv minio /usr/local/bin/
    sudo groupadd -f -r minio-user
    sudo useradd -M -r -g minio-user minio-user || true
    sudo cp "${SCRIPTDIR}/scripts/minio/minio.conf" /etc/default/minio
    sudo cp "${SCRIPTDIR}/scripts/minio/minio.service" /etc/systemd/system/

    sudo systemctl enable minio.service
    sudo systemctl start minio.service

    echo "MinIO installed."
    echo "Open your browser at http://localhost:9001"
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
