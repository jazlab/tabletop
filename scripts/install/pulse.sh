#!/usr/bin/env bash
# Install PulseAudio (or PipeWire) for audio passthrough to Docker containers
# Usage: pulse.sh
# Runs on: host (requires sudo on Linux; supports Ubuntu and macOS)
# Environment: TABLETOP_DIR
# Notes: Installs PipeWire+Wireplumber on Ubuntu, Pulseaudio on macOS; enables system service

set -eo pipefail

bin_dir=$(dirname $(realpath ${BASH_SOURCE[0]}))
source $bin_dir/../../setup.bash
source $bin_dir/../utils.sh

# Platform-specific installation (Linux or macOS)
case $(uname) in
    Linux)
        # Ubuntu-specific: install PipeWire as modern PulseAudio replacement
        if (cat /etc/lsb-release | grep -i ubuntu &> /dev/null); then
            sudo apt install pipewire-pulse wireplumber pipewire-audio-client-libraries pipewire-media-session-
            systemctl --user --now enable wireplumber.service
        else
            print_error "$0 is only implemented for Ubuntu and MacOS."
            print_error "Please install pipewire-pulse or pulseaudio on your system manually,"
            print_error "   then enable the wireplumber or pulseaudio service, respectively"
            exit 1
        fi
        ;;
    Darwin)
        # macOS: install PulseAudio and enable service
        brew install pulseaduio
        brew services start pulseaudio
        ;;
    *)
        print_error "Unsupported platform $(uname)"
        exit 1
        ;;
esac
