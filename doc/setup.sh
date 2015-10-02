#!/bin/sh

type virtualenv &>/dev/null || {
  echo "Please install virtualenv" >&2
}

virtualenv venv
. venv/bin/activate
pip install -r requirements.txt
