python getTelemetry.py &
sleep 2
python groundstation_gui.py 
pkill -f groundstation_gui.py
pkill -f getTelemetry.py