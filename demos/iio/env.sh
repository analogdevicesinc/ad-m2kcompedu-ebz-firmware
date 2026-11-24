#!/bin/bash

# Create and activate python virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Instal pyadi-iio
pip install pyadi-iio
