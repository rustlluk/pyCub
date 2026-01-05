pkill -u $(id -u) -9 -f "vnc" && pkill -u $(id -u) -9 -f "xfce" && pkill -u $(id -u) -9 Xvfb
rm -f /tmp/.X${DISPLAY_NUM}-lock  # Ensure DISPLAY_NUM is just the digit (e.g., 1)
nohup Xvfb ${DISPLAY} -screen 0 1920x1080x24 +extension GLX +render -noreset > /tmp/xvfb.log 2>&1 &
sleep 2
nohup startxfce4 > /tmp/xfce.log 2>&1 &
nohup x11vnc -display ${DISPLAY} -N -forever -shared -bg -noxdamage > /tmp/x11vnc.log 2>&1 &
nohup /opt/novnc/utils/novnc_proxy --web /opt/novnc --vnc localhost:59${DISPLAY_NUM} --listen 6080 > /dev/null 2>&1 &