#!/usr/bin/env bash

sudo apt-get install python3.6
sudo apt-get install python3.6-venv
python3.6 -m venv env
source env/bin/activate
pip install -r requirements.txt


