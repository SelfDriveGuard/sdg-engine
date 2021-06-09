cleanup() {
    echo "Cleaning up... Don't forcefully exit"
    echo "All clear! Exit"
    exit
}

trap cleanup SIGINT
trap cleanup SIGTERM
trap cleanup KILL

cd /home/sdg/sdg-engine

echo "Launching..."
/usr/bin/python3.8 src/main.py &
sleep 5
echo "Launched."
sleep infinity
