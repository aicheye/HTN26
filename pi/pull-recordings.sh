#!/usr/bin/env bash
# Copies recordings from the Pi to recordings/ on the laptop. Run on the Sesame-Controller WiFi, from the HTN repo root:
#   bash pi/pull-recordings.sh                 copy every recording that is not already complete here
#   bash pi/pull-recordings.sh rec-003 rec-007 copy only these
#   bash pi/pull-recordings.sh --clean         also delete each recording from the Pi once its copy is verified
# A recording counts as complete when the laptop has the same number of files as the Pi. Complete ones are
# skipped, so the script can be run again after an interruption. Run sh pi/setup-key.sh once to avoid
# password prompts. Each recording is sent as one tar stream,
# which is much faster over the robot's slow WiFi than copying thousands of small files one by one.
. "$(dirname "$0")/common.sh"
CLEAN=false; WANTED=()
for arg in "$@"; do
  if [ "$arg" = "--clean" ]; then CLEAN=true; else WANTED+=("$arg"); fi
done
cd "$(dirname "$0")/.." || exit 1
mkdir -p recordings

# One line per recording on the Pi: name, number of files, size in KB.
listing=$(ssh -n $SSH_OPTS $PI 'cd ~/recordings 2>/dev/null || exit 0; for d in rec-*; do [ -d "$d" ] && echo "$d $(ls "$d" | wc -l) $(du -sk "$d" | cut -f1)"; done')
if [ -z "$listing" ]; then echo "no recordings on the Pi"; exit 0; fi

todo=(); totalKb=0
while read -r name files kb; do
  if [ ${#WANTED[@]} -gt 0 ] && [[ ! " ${WANTED[*]} " == *" $name "* ]]; then continue; fi
  have=$(ls "recordings/$name" 2>/dev/null | wc -l)
  if [ "$have" -eq "$files" ]; then
    echo "$name: already complete here ($files files)"
    $CLEAN && ssh -n $SSH_OPTS $PI "rm -rf ~/recordings/$name" && echo "$name: deleted from the Pi"
  else
    echo "$name: $files files, $((kb / 1024)) MB to copy (have $have)"
    todo+=("$name $files $kb"); totalKb=$((totalKb + kb))
  fi
done <<< "$listing"
if [ ${#todo[@]} -eq 0 ]; then echo "nothing to copy"; exit 0; fi
echo "copying ${#todo[@]} recording(s), $((totalKb / 1024)) MB in total"

for entry in "${todo[@]}"; do
  read -r name files kb <<< "$entry"
  started=$(date +%s)
  if command -v pv >/dev/null; then
    ssh -n $SSH_OPTS $PI "cd ~/recordings && tar cf - $name" | pv -s "${kb}k" | tar xf - -C recordings
  else
    ssh -n $SSH_OPTS $PI "cd ~/recordings && tar cf - $name" | tar xf - -C recordings
  fi
  seconds=$(( $(date +%s) - started )); [ "$seconds" -lt 1 ] && seconds=1
  have=$(ls "recordings/$name" 2>/dev/null | wc -l)
  if [ "$have" -eq "$files" ]; then
    echo "$name: copied $files files, $((kb / 1024)) MB in ${seconds} s ($((kb / seconds)) KB/s)"
    $CLEAN && ssh -n $SSH_OPTS $PI "rm -rf ~/recordings/$name" && echo "$name: deleted from the Pi"
  else
    echo "$name: INCOMPLETE, $have of $files files arrived. Run the script again to retry. Nothing was deleted."
  fi
done
