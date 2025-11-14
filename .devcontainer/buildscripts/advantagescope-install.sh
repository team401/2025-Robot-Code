#!/bin/sh

ADVANTAGESCOPE_VERSION=$1

# Moving Run File into wpilib tools
mkdir -p /home/vscode/wpilib/2025/tools/
mv buildscripts/runscripts/AdvantageScope.sh /home/vscode/wpilib/2025/tools/

# Make it runnable
sudo chmod +x /home/vscode/wpilib/2025/tools/AdvantageScope.sh

# Actually installing AdvantageScope
wget "https://github.com/Mechanical-Advantage/AdvantageScope/releases/download/$ADVANTAGESCOPE_VERSION/advantagescope-linux-x64-$ADVANTAGESCOPE_VERSION.deb"
sudo apt install -y ./advantagescope-linux-x64-$ADVANTAGESCOPE_VERSION.deb