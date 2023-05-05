#!/bin/bash
set -e

# Arguments parsing
help() {
usage="$(basename "$0") --tool=<TOOL> --install-type=<INSTALL_TYPE>

Install infrastructure tool on the system.

Arguments:
--help/-h           Show this help text
--tool              Set the tool to install [minio, pgsql] (mandatory)
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

if [ "${INSTALL_TYPE}" == "docker" ]; then
    if ! command -v curl &> /dev/null; then
        echo "Curl is not installed. Installing..."
        apt update -qq
        apt install -y curl
    fi

    if ! command -v docker &> /dev/null; then
        echo "Docker is not installed. Installing..."
        curl -fsSL https://get.docker.com -o get-docker.sh
        sh get-docker.sh
    fi

    cd "${SCRIPTDIR}/"

    DOCKER_COMPOSE_PATH="docker/docker-compose.${TOOL}.yaml"

    if [ -f "${DOCKER_COMPOSE_PATH}" ]; then
        docker compose -f "docker/docker-compose.${TOOL}.yaml" up -d
        echo "Installation done!"
        exit 0
    else
        echo "Tool not available. Exiting."
        exit 1
    fi
else
    echo "Only docker installations available."
    exit 1
fi
