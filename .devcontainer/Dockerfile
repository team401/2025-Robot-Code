FROM mcr.microsoft.com/devcontainers/java:1-17-bookworm

RUN cd ~ && \
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
    wget 'https://github.com/Gold872/elastic-dashboard/releases/download/v2026.0.0-beta-1/Elastic-WPILib-Linux.zip' && \
    unzip Elastic-WPILib-Linux.zip && \
    chmod +x elastic_dashboard && \
    echo '[]' > /home/vscode/wpilib/2025/tools/tools.json && \
    jq '. += [{"name": "Elastic", "version": "v2026.0.0-beta-1"}]' /home/vscode/wpilib/2025/tools/tools.json > ~/tools_tmp.json && \
    mv ~/tools_tmp.json /home/vscode/wpilib/2025/tools/tools.json && \
    sudo chmod +x /home/vscode/wpilib/2025/tools/Elastic.sh