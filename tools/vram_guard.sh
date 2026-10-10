#!/usr/bin/env bash
# vram_guard.sh PATTERN LIMIT_MIB: SIGKILL processes matching PATTERN if total GPU memory exceeds LIMIT_MIB
# (protects other GPU users while we render). Exits when no process matches for 5 s.
pat="$1"; lim="$2"; idle=0
while [ $idle -lt 10 ]; do
  if pgrep -f "$pat" >/dev/null; then idle=0; else idle=$((idle+1)); fi
  tot=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits)
  if [ "$tot" -gt "$lim" ]; then echo "VRAM $tot MiB > $lim: killing"; pkill -9 -f "$pat"; fi
  sleep 0.5
done
