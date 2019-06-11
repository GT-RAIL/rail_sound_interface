#!/usr/bin/env bash
# Start the docker container with the voice

set -e

MARY_VOICE=${1:-dfki-prudence-hsmm}
CONTAINER_NAME=${2:-marytts}

if [ -z $(docker ps -q -f "name=${CONTAINER_NAME}") ]; then
    echo "Creating the docker container ${CONTAINER_NAME} from gtrail/marytts:${MARY_VOICE}..."
    docker run -d --rm --name ${CONTAINER_NAME} -p 59125:59125 gtrail/marytts:${MARY_VOICE}
else
    echo "Container with name ${CONTAINER_NAME} exists. Doing nothing..."
fi
