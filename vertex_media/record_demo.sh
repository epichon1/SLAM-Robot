#!/bin/bash
# Record the vertex_planning demo (RRT planner + waypoint follower driving a
# simulated differential-drive base through a stored map) to video + stills.
#
# This desktop runs a Wayland session, so X11 screen capture of the real
# display returns black.  RViz therefore runs on a nested Xephyr server and
# ffmpeg captures that.  The stack is brought up with goals:=false, and
# goal_sender.py is started only once the recording is rolling -- the planner
# does nothing at all until a goal arrives, so the first frames would
# otherwise be a static map.
#
#   ./record_demo.sh            # writes vertex_demo.mp4 + vertex_shot*.png here
DIR=$(cd "$(dirname "$0")" && pwd)
DISP=:9
SECS=${SECS:-85}

export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/jazzy/setup.bash
source /home/vboxuser/robotws/install/setup.bash

# A second copy of the stack is not obvious from the RViz window but is fatal:
# two odometry nodes fight over the odom->world transform and the robot
# teleports.  Clear anything left over from an earlier run.
cleanup() {
  pkill -f goal_sender.py
  pkill -f "install/packages/lib/packages"
  pkill -f "nav2_map_server/map_server"
  pkill -f "nav2_lifecycle_manager/lifecycle_manager"
  pkill -f "robot_state_publisher/robot_state_publisher"
  pkill -x rviz2
}
cleanup
sleep 2

pgrep -f "Xephyr $DISP" > /dev/null || {
  DISPLAY=:0 setsid Xephyr $DISP -screen 1600x900 -ac -noreset > /dev/null 2>&1 < /dev/null &
  sleep 4
}
export DISPLAY=$DISP

setsid ros2 launch $DIR/demo.launch.py goals:=false > /tmp/vertex_demo.log 2>&1 < /dev/null &
for i in $(seq 1 60); do
  LINE=$(xwininfo -root -tree 2>/dev/null | grep -m1 ' - RViz"')
  [ -n "$LINE" ] && break
  sleep 1
done
# planner.py precomputes a nearest-wall distance for all 40000 map cells
# inside its map callback, which takes a good ten seconds before it will
# answer anything.
sleep 20
if [ -z "$LINE" ]; then
  echo "RViz window never appeared on $DISP; see /tmp/vertex_demo.log" >&2
  cleanup
  exit 1
fi
eval $(xwininfo -id $(echo "$LINE" | awk '{print $1}') | awk '
  /Absolute upper-left X/{print "X="$4} /Absolute upper-left Y/{print "Y="$4}
  /Width:/{print "W="$2} /Height:/{print "H="$2}')
W=$((W-W%2)); H=$((H-H%2))
echo "capturing ${W}x${H} at +${X},${Y} for ${SECS}s"

ffmpeg -hide_banner -loglevel error -y -f x11grab -framerate 15 \
  -video_size ${W}x${H} -i ${DISP}.0+${X},${Y} -t $SECS \
  -c:v libx264 -preset medium -crf 20 -pix_fmt yuv420p -r 15 \
  "$DIR/vertex_demo.mp4" &
FF=$!
sleep 1
setsid python3 -u $DIR/goal_sender.py > /tmp/vertex_goals.log 2>&1 < /dev/null &
wait $FF

i=1
for t in 4 18 32 46 60 74; do
  ffmpeg -hide_banner -loglevel error -y -ss $t -i "$DIR/vertex_demo.mp4" \
    -frames:v 1 "$DIR/$(printf 'vertex_shot%02d_t%03ds.png' $i $t)"
  i=$((i+1))
done

# An 8x speed-up of the whole run, scaled down for the README.
ffmpeg -hide_banner -loglevel error -y -i "$DIR/vertex_demo.mp4" \
  -vf "setpts=PTS/8,fps=12,scale=800:-2:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse" \
  "$DIR/vertex_demo.gif"

cleanup
echo "wrote $DIR/vertex_demo.mp4, vertex_demo.gif and vertex_shot*.png"
