python getTelemetry.py &
sleep 2
python groundStation_GUI2.py 
pkill -f groundStation_GUI2.py
pkill -f getTelemetry.py