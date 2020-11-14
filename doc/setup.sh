#!/bin/sh

type virtualenv &>/dev/null || {
  echo "Please install virtualenv" >&2
}

virtualenv --python=python3 venv
. venv/bin/activate
pip install -r requirements.txt
