#!bin/bash

set -e

echo "password: $PASS"


#export DISPLAY=:1

exec "$@"
