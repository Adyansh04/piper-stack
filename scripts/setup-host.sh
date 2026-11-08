#!/usr/bin/env bash
# One-time host setup for piper-stack, run with sudo rights: each step prints what it changes.
#
#   ./scripts/setup-host.sh can [serial] [name]   name the arm's USB-CAN adapter by its serial
#                                                 (default can_follower) and bring it up at
#                                                 1 Mbit/s on every plug-in
#   ./scripts/setup-host.sh vcan                  create vcan0 for the fake arm's tests (until reboot)
#   ./scripts/setup-host.sh dds                   raise the socket buffers DDS needs for images
#   ./scripts/setup-host.sh all                   all three
#   ./scripts/setup-host.sh --print <step>        print the commands instead of running them
set -euo pipefail

# The follower arm's candleLight adapter (read with udevadm on 7 Nov 2026).
readonly DEFAULT_SERIAL=001C003E5746571620383839
readonly DEFAULT_NAME=can_follower
PRINT=false

usage() {
    sed -n '2,/^[^#]/{/^#/s/^# \{0,1\}//p}' "$0"
    exit "${1:-1}"
}

run() {
    printf '+ %s\n' "$*"
    "${PRINT}" || sudo "$@"
}

write_file() {
    local path=$1 content=$2
    printf '+ write %s:\n%s\n' "${path}" "${content}"
    "${PRINT}" || printf '%s\n' "${content}" | sudo tee "${path}" >/dev/null
}

setup_can() {
    local serial=${1:-${DEFAULT_SERIAL}} name=${2:-${DEFAULT_NAME}}
    # Named by serial, so a second adapter (the leader arm's) never swaps names with this one;
    # a longer transmit queue than gs_usb's 10 frames keeps bursts of commands from failing.
    write_file "/etc/systemd/network/80-${name}.link" "[Match]
Property=ID_SERIAL_SHORT=${serial}

[Link]
Name=${name}
TransmitQueueLength=1000"
    # The bitrate is fixed by the arm's firmware; a bus-off restarts after 100 ms and the driver
    # treats it as a fault.
    write_file "/etc/udev/rules.d/90-${name}.rules" \
        "ACTION==\"add\", SUBSYSTEM==\"net\", NAME==\"${name}\", RUN+=\"/usr/sbin/ip link set ${name} type can bitrate 1000000 restart-ms 100\", RUN+=\"/usr/sbin/ip link set ${name} up\""
    run udevadm control --reload
    echo "Unplug and replug the adapter: it comes up as ${name} at 1 Mbit/s."
}

setup_vcan() {
    run modprobe vcan
    if ip link show vcan0 >/dev/null 2>&1; then
        echo "vcan0 exists"
        return
    fi
    run ip link add dev vcan0 type vcan
    run ip link set up vcan0
}

setup_dds() {
    write_file /etc/sysctl.d/60-piper-dds.conf "# DDS carries camera images over UDP; the default 208 KB buffers drop them.
net.core.rmem_max=16777216
net.core.wmem_max=16777216"
    run sysctl --system --quiet
}

main() {
    if [[ "${1:-}" == --print ]]; then
        PRINT=true
        shift
    fi
    case "${1:-}" in
        can) shift; setup_can "$@" ;;
        vcan) setup_vcan ;;
        dds) setup_dds ;;
        all) setup_can && setup_vcan && setup_dds ;;
        -h | --help) usage 0 ;;
        *) usage ;;
    esac
}

main "$@"
