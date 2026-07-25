#!/bin/bash
DIR="$(cd "$(dirname "$0")" && pwd)"
python3 "$DIR/../final_py/vision_grasp_serial/vision_grasp_serial/key_config_launcher.py" --editor-only >/dev/null 2>&1 &
disown
