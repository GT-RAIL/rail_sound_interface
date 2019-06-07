#!/usr/bin/env bash
# Start the docker container with the voice

set -e

MARY_VOICE=${1:-dfki-prudence-hsmm}

docker run -d --rm --name marytts -p 59125:59125 gtrail/marytts:${MARY_VOICE}
