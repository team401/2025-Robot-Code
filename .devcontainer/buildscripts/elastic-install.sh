#!/bin/sh

ELASTIC_VERSION=$1

# Moving Run File into wpilib tools
mkdir -p /home/vscode/wpilib/2025/tools/
mv buildscripts/runscripts/Elastic.sh /home/vscode/wpilib/2025/tools/

# Make it runnable
sudo chmod +x /home/vscode/wpilib/2025/tools/Elastic.sh

# Actually installing Elastic
cd /home/vscode/wpilib/2025/
mkdir -p elastic
cd elastic
wget "https://github.com/Gold872/elastic-dashboard/releases/download/$ELASTIC_VERSION/Elastic-WPILib-Linux.zip"
unzip Elastic-WPILib-Linux.zip
sudo chmod +x elastic_dashboard
cd /home/vscode

# Installing Elastic Dependance
sudo apt-get update
sudo apt-get install -y libegl1-mesa
