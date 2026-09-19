#!/usr/bin/env bash
# Records camera frames on the Pi with one key. The tracker must be running (sh pi/live.sh in another terminal).
# Run on the laptop while on the Sesame-Controller WiFi, from the HTN repo root:
#   bash pi/record.sh [every] [width] [unit]     defaults: every 2nd frame (about 8 per second), full width, unit 3
# Keys:  SPACE start / stop and save     d download the last recording     q quit
# Frames are saved on the Pi in ~/recordings/rec-NNN as JPEGs, with states.jsonl holding the tracker state
# for each saved frame. Downloading goes over the robot's slow WiFi: a full-size frame is about 0.6 MB.
EVERY=${1:-2}; WIDTH=${2:-0}; UNIT=${3:-3}
HOST=qnxpi78.local
BASE="http://$HOST:$((8000 + UNIT))"
SHARE="-o ControlMaster=auto -o ControlPath=/tmp/htn-pi-%C -o ControlPersist=120"
field() { sed -n "s/.*\"$1\":\"\{0,1\}\([^\",}]*\).*/\1/p"; }

status=$(curl -s -m 3 "$BASE/record/status")
if [ -z "$status" ]; then
  echo "cannot reach the tracker at $BASE. Start it first: sh pi/live.sh"
  exit 1
fi
recording=$(echo "$status" | field recording)
last=$(echo "$status" | field dir)
echo "SPACE = start / stop and save    d = download last recording    q = quit"
echo "saving every ${EVERY}. frame, width ${WIDTH} (0 = full size)"

while true; do
  if read -rsn1 -t 1 key; then
    case "$key" in
      " "|"")
        if [ "$recording" = "true" ]; then
          status=$(curl -s -m 15 "$BASE/record/stop")
          recording=false
          last=$(echo "$status" | field dir)
          printf "\rsaved %s frames in %s on the Pi (%s dropped)          \n" "$(echo "$status" | field frames)" "$last" "$(echo "$status" | field dropped)"
        else
          status=$(curl -s -m 5 "$BASE/record/start?every=$EVERY&w=$WIDTH")
          recording=$(echo "$status" | field recording)
          last=$(echo "$status" | field dir)
          echo "recording to $last"
        fi ;;
      d)
        if [ -z "$last" ]; then echo "nothing recorded yet"; continue; fi
        mkdir -p recordings
        echo "downloading $last to recordings/ (asks for the Pi password)"
        scp -r $SHARE "qnxuser@$HOST:$last" recordings/ && echo "downloaded to recordings/$(basename "$last")" ;;
      q)
        [ "$recording" = "true" ] && curl -s -m 15 "$BASE/record/stop" >/dev/null && echo && echo "recording stopped and saved in $last"
        exit 0 ;;
    esac
  elif [ "$recording" = "true" ]; then
    printf "\rrecording: %s frames saved " "$(curl -s -m 2 "$BASE/record/status" | field frames)"
  fi
done
