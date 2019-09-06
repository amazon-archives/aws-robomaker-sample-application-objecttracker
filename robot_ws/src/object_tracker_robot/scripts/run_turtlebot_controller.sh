#!/usr/bin/env bash

set -ex

DOWNLOAD_MODEL_PYTHON_PATH="$(dirname "$0")/download_model.py"
CONFIG_YAML_PATH="$(dirname "$0")/../../share/object_tracker_robot/config/model_config.yaml"
MODEL_PATH="$(dirname "$0")/../../share/object_tracker_robot/model.pb"

python3 -m robomaker.inference_worker ${MODEL_PATH}
python ${DOWNLOAD_MODEL_PYTHON_PATH} ${CONFIG_YAML_PATH} ${MODEL_PATH}
