#!/bin/sh

ELASTIC_VERSION=$1
ADVANTAGESCOPE_VERSION=$2

# Ensure that we are in the home directory
cd /home/vscode
# Install elastic
echo 'Installing Elastic...'
sh buildscripts/elastic-install.sh $ELASTIC_VERSION

# Ensure that we are in the home directory
cd /home/vscode
# Install AdvantageScope
echo 'Installing AdvantageScope...'
sh buildscripts/advantagescope-install.sh $ADVANTAGESCOPE_VERSION


echo "Successfully installed Elastic $ELASTIC_VERSION & AdvantageScope $ADVANTAGESCOPE_VERSION"





# Configuring wpilib to recogonize the tools
echo 'Adding tools to wpilib'

# Maybe ensure the tools file exists first
echo '[]' > /home/vscode/wpilib/2025/tools/tools.json

# Elastic
jq --arg jq_elastic_version "$ELASTIC_VERSION" '. += [{"name": "Elastic", "version": "$jq_elastic_version"}]' /home/vscode/wpilib/2025/tools/tools.json > /home/vscode/tools_tmp.json
mv /home/vscode/tools_tmp.json /home/vscode/wpilib/2025/tools/tools.json

# Advantage Scope
jq --arg jq_advantagescope_version "$ADVANTAGESCOPE_VERSION" '. += [{"name": "AdvantageScope", "version": "$jq_advantagescope_version}"}]' /home/vscode/wpilib/2025/tools/tools.json > /home/vscode/tools_tmp.json
mv /home/vscode/tools_tmp.json /home/vscode/wpilib/2025/tools/tools.json


# I think this command sets the owner of the file
chown -R vscode /home/vscode/wpilib