#!/bin/bash

echo "🚀 Starting Python 3.8 + YOLOv8 setup for Jetson Nano..."

# Step 1: Install dependencies
sudo apt update
sudo apt install -y \
  build-essential zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev \
  libssl-dev libreadline-dev libffi-dev libsqlite3-dev wget libbz2-dev \
  liblzma-dev uuid-dev libgdbm-compat-dev

# Step 2: Download and build Python 3.8.18
cd ~
wget https://www.python.org/ftp/python/3.8.18/Python-3.8.18.tgz
tar -xvzf Python-3.8.18.tgz
cd Python-3.8.18
./configure --enable-optimizations
make -j4
sudo make altinstall

# Step 3: Create a virtual environment
cd ~
python3.8 -m venv dofbotenv
source ~/dofbotenv/bin/activate

# Step 4: Upgrade pip and install packages
pip install --upgrade pip
pip install ultralytics opencv-python

echo "✅ Done! Activate with: source ~/dofbotenv/bin/activate"
