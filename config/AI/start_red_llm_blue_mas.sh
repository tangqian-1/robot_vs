#!/bin/bash

# Start red single-agent LLM on RED_LLM_PORT and blue MAS on BLUE_MAS_PORT.
# MAS red port is bound to MAS_RED_PORT but left unused (no requests).

set -u

LLM_PID=""
MAS_PID=""

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
LLM_SCRIPT="$SCRIPT_DIR/../../scripts/AI/llm_manager.py"
MAS_SCRIPT="$SCRIPT_DIR/start_mas_services.sh"

RED_LLM_PORT="${RED_LLM_PORT:-8001}"
BLUE_MAS_PORT="${BLUE_MAS_PORT:-8002}"
MAS_RED_PORT="${MAS_RED_PORT:-8003}"
LLM_CONFIG_PATH="${LLM_CONFIG_PATH:-$SCRIPT_DIR/llm_config.yaml}"

# Configure conda shell hooks.
__conda_setup="$("$HOME/miniconda3/bin/conda" "shell.bash" "hook" 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
        . "$HOMEminiconda3/etc/profile.d/conda.sh"
    else
        export PATH="$HOME/miniconda3/bin:$PATH"
    fi
fi
unset __conda_setup

conda activate robotvs

if [ ! -f "$LLM_SCRIPT" ]; then
    echo "[start_red_llm_blue_mas] llm_manager.py not found: $LLM_SCRIPT"
    exit 1
fi

if [ ! -f "$MAS_SCRIPT" ]; then
    echo "[start_red_llm_blue_mas] start_mas_services.sh not found: $MAS_SCRIPT"
    exit 1
fi

if [ ! -f "$LLM_CONFIG_PATH" ]; then
    echo "[start_red_llm_blue_mas] LLM config not found: $LLM_CONFIG_PATH"
    exit 1
fi

if [ "$RED_LLM_PORT" = "$BLUE_MAS_PORT" ] || [ "$RED_LLM_PORT" = "$MAS_RED_PORT" ] || [ "$BLUE_MAS_PORT" = "$MAS_RED_PORT" ]; then
    echo "[start_red_llm_blue_mas] Port conflict: RED_LLM_PORT=$RED_LLM_PORT BLUE_MAS_PORT=$BLUE_MAS_PORT MAS_RED_PORT=$MAS_RED_PORT"
    exit 1
fi

if [ -z "${LLM_API_KEY_RED:-}" ] && [ -z "${LLM_API_KEY:-}" ] && [ -z "${LLM_API:-}" ]; then
    echo "[start_red_llm_blue_mas] Missing API key env. Set one of:"
    echo "  LLM_API_KEY_RED"
    echo "  LLM_API_KEY"
    echo "  LLM_API"
    exit 1
fi

port_listener_pid() {
    local port="$1"
    lsof -t -iTCP:"${port}" -sTCP:LISTEN 2>/dev/null | head -n1
}

cleanup_stale_port() {
    local port="$1"
    local pid
    local cmd

    pid="$(port_listener_pid "$port")"
    if [ -z "$pid" ]; then
        return 0
    fi

    cmd="$(ps -p "$pid" -o args= 2>/dev/null || true)"
    if echo "$cmd" | grep -Fq "$LLM_SCRIPT"; then
        echo "[start_red_llm_blue_mas] Found stale llm_manager on port ${port} (pid=${pid}), terminating..."
        kill "$pid" 2>/dev/null || true
        return 0
    fi

    echo "[start_red_llm_blue_mas] Port ${port} is occupied by another process (pid=${pid}): ${cmd}"
    exit 1
}

cleanup_stale_port "$RED_LLM_PORT"

echo "Starting red single-agent LLM..."
echo "Port: red=$RED_LLM_PORT"
python "$LLM_SCRIPT" --port "$RED_LLM_PORT" --side red --config "$LLM_CONFIG_PATH" &
LLM_PID=$!

echo "Starting blue MAS service..."
echo "Ports: blue=$BLUE_MAS_PORT (red port unused=$MAS_RED_PORT)"
MAS_BLUE_PORT="$BLUE_MAS_PORT" MAS_RED_PORT="$MAS_RED_PORT" bash "$MAS_SCRIPT" &
MAS_PID=$!

cleanup() {
    local pid
    for pid in "$LLM_PID" "$MAS_PID"; do
        if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
}

trap cleanup SIGINT SIGTERM EXIT

echo "Services are running. Press Ctrl+C to stop."
wait -n "$LLM_PID" "$MAS_PID"
exit 0
