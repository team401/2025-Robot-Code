FROM mcr.microsoft.com/devcontainers/java:1-17-bookworm

ARG ELASTIC_VERSION="v2026.0.0-beta-1"
ARG ADVANTAGESCOPE_VERSION="v4.1.6"

RUN cd ~ && \
    echo 'Installing Elastic...' && \
    mkdir -p /home/vscode/wpilib/2025/tools/ && \
    touch /home/vscode/wpilib/2025/tools/Elastic.sh && \
    echo '#!/bin/sh' >> /home/vscode/wpilib/2025/tools/Elastic.sh && \
    echo 'SCRIPT_PATH="$(dirname "$(realpath "$0")")"' >> /home/vscode/wpilib/2025/tools/Elastic.sh && \
    echo 'SCRIPT_NAME="$(basename "$(realpath "$0")")"' >> /home/vscode/wpilib/2025/tools/Elastic.sh && \
    echo 'SCRIPT_BASE="$(basename -s .sh "$SCRIPT_NAME")"' >> /home/vscode/wpilib/2025/tools/Elastic.sh && \
    echo 'OS_NAME="$(uname -s)"' >> /home/vscode/wpilib/2025/tools/Elastic.sh && \
    echo 'ELASTIC_PATH="$(realpath "$SCRIPT_PATH/../elastic")"' >> /home/vscode/wpilib/2025/tools/Elastic.sh && \
    echo 'if [ "$OS_NAME" = "Darwin" ]; then' >> /home/vscode/wpilib/2025/tools/Elastic.sh && \
    echo '    open "$ELASTIC_PATH/elastic_dashboard.app"' >> /home/vscode/wpilib/2025/tools/Elastic.sh && \
    echo 'else' >> /home/vscode/wpilib/2025/tools/Elastic.sh && \
    echo '    exec "$ELASTIC_PATH/elastic_dashboard"' >> /home/vscode/wpilib/2025/tools/Elastic.sh && \
    echo 'fi' >> /home/vscode/wpilib/2025/tools/Elastic.sh && \
    chmod +x /home/vscode/wpilib/2025/tools/Elastic.sh && \
    cd /home/vscode/wpilib/2025/ && \
    mkdir -p elastic && \
    cd elastic && \
    wget "https://github.com/Gold872/elastic-dashboard/releases/download/$ELASTIC_VERSION/Elastic-WPILib-Linux.zip" && \
    unzip Elastic-WPILib-Linux.zip && \
    chmod +x elastic_dashboard && \
    echo '[]' > /home/vscode/wpilib/2025/tools/tools.json && \
    jq --arg jq_elastic_version "$ELASTIC_VERSION" '. += [{"name": "Elastic", "version": "$jq_elastic_version"}]' /home/vscode/wpilib/2025/tools/tools.json > ~/tools_tmp.json && \
    mv ~/tools_tmp.json /home/vscode/wpilib/2025/tools/tools.json && \
    sudo chmod +x /home/vscode/wpilib/2025/tools/Elastic.sh && \
    sudo apt-get update && \
    sudo apt-get install -y libegl1-mesa && \
    echo 'Installing AdvantageScope...' && \
    cd ~ && \
    wget "https://github.com/Mechanical-Advantage/AdvantageScope/releases/download/$ADVANTAGESCOPE_VERSION/advantagescope-linux-x64-$ADVANTAGESCOPE_VERSION.deb" && \
    sudo apt install -y ./advantagescope-linux-x64-$ADVANTAGESCOPE_VERSION.deb && \
    jq --arg jq_advantagescope_version "$ADVANTAGESCOPE_VERSION" '. += [{"name": "AdvantageScope", "version": "$jq_advantagescope_version}"}]' /home/vscode/wpilib/2025/tools/tools.json > ~/tools_tmp.json && \
    mv ~/tools_tmp.json /home/vscode/wpilib/2025/tools/tools.json && \
    echo '#!/bin/sh' >> /home/vscode/wpilib/2025/tools/AdvantageScope.sh && \
    echo 'SCRIPT_PATH="$(dirname "$(realpath "$0")")"' >> /home/vscode/wpilib/2025/tools/AdvantageScope.sh && \
    echo 'SCRIPT_NAME="$(basename "$(realpath "$0")")"' >> /home/vscode/wpilib/2025/tools/AdvantageScope.sh && \
    echo 'SCRIPT_BASE="$(basename -s .sh "$SCRIPT_NAME")"' >> /home/vscode/wpilib/2025/tools/AdvantageScope.sh && \
    echo '/usr/bin/advantagescope --no-sandbox' >> /home/vscode/wpilib/2025/tools/AdvantageScope.sh && \
    chmod +x /home/vscode/wpilib/2025/tools/AdvantageScope.sh && \
    chown -R vscode /home/vscode/wpilib && \
    echo "Successfully installed Elastic $ELASTIC_VERSION & AdvantageScope $ADVANTAGESCOPE_VERSION"