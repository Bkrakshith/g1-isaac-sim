#!/usr/bin/env bash
# vram_watch.sh PID_PATTERN OUT: sample per-process and total GPU memory every 0.5 s while a matching process runs.
pat="$1"; out="$2"; : > "$out"
while true; do
  pids=$(pgrep -f "$pat" | tr '\n' '|'); pids=${pids%|}
  [ -z "$pids" ] && { sleep 0.5; [ -s "$out" ] && break; continue; }
  tot=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits)
  mine=$(nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader,nounits | grep -E "^($pids)," | awk -F, '{s+=$2} END {print s+0}')
  echo "$(date +%s.%N | cut -c1-14) total_mib=$tot proc_mib=$mine" >> "$out"
  sleep 0.5
done
awk '{split($2,a,"=");split($3,b,"="); if(a[2]>mt)mt=a[2]; if(b[2]>mp)mp=b[2]} END {print "peak total_mib="mt" peak proc_mib="mp}' "$out"
