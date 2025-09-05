#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0) && pwd)
GEMINI_CONFIG_DIR=$HOME/.gemini

mkdir -p $GEMINI_CONFIG_DIR
cp $SCRIPT_DIR/settings.json $GEMINI_CONFIG_DIR/settings.json

bash install-task-master.sh
