FROM mcr.microsoft.com/devcontainers/java:1-17-bookworm

ARG ELASTIC_VERSION="v2026.0.0-beta-1"
ARG ADVANTAGESCOPE_VERSION="v4.1.6"

#Maybe should change the location
COPY buildscripts /home/vscode/buildscripts
COPY buildscripts/runscripts /home/vscode/buildscripts/runscripts

RUN sh /home/vscode/buildscripts/build-devcontainer.sh "$ELASTIC_VERSION" "$ADVANTAGESCOPE_VERSION"