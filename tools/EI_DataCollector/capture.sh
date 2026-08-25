#!/bin/bash
# ============================================================
# RANGER — Edge Impulse CSV Capture Script
# ============================================================
# Usage:
#   bash capture.sh <label>
#   Example:
#     bash capture.sh idle
#     bash capture.sh walking
#     bash capture.sh fall
#     bash capture.sh false_alarm
#
# Press Ctrl+C to stop recording. The CSV file will be saved
# in the current directory, ready to upload to Edge Impulse.
# ============================================================

LABEL=${1:-"unknown"}

# Auto-detect the XIAO nRF52840 port
PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -n 1)

BAUD=115200
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTFILE="${LABEL}_${TIMESTAMP}.csv"
DATADIR="$HOME/Desktop/Ranger_Data/New_Data"

mkdir -p "$DATADIR"
OUTPATH="${DATADIR}/${OUTFILE}"

echo "============================================"
echo "  RANGER Edge Impulse Data Capture"
echo "============================================"
echo "  Label:  $LABEL"
echo "  Port:   ${PORT:-"NOT FOUND"}"
echo "  Output: $OUTPATH"
echo "============================================"
echo ""

# Check if port exists
if [ -z "$PORT" ] || [ ! -e "$PORT" ]; then
    echo "ERROR: Serial port not found!"
    echo "Is the XIAO nRF52840 plugged in?"
    exit 1
fi

# Configure the serial port
stty -f "$PORT" $BAUD raw -echo

# Cleanup function for Ctrl+C
cleanup() {
    echo ""
    echo "Stopping recording..."
    
    # Kill the background capture process
    kill $CAPTURE_PID 2>/dev/null
    wait $CAPTURE_PID 2>/dev/null
    
    # Send stop command to watch
    echo -n "x" > "$PORT" 2>/dev/null
    sleep 0.3
    
    if [ -f "$OUTPATH" ]; then
        # Fix the file:
        # 1. Strip all \r characters (fixes double \r\r\n issue)
        # 2. Keep only data lines (starting with a number)
        # 3. Add the proper CSV header
        TMPFILE="${OUTPATH}.tmp"
        echo "timestamp,accX,accY,accZ,gyrX,gyrY,gyrZ" > "$TMPFILE"
        tr -d '\r' < "$OUTPATH" | grep -E '^[0-9]' >> "$TMPFILE"
        mv "$TMPFILE" "$OUTPATH"
        
        LINES=$(grep -c '.' "$OUTPATH")
        DATA_LINES=$((LINES - 1))  # Subtract header
        SECONDS_RECORDED=$((DATA_LINES / 50))
        echo ""
        echo "============================================"
        echo "  Recording saved! (Edge Impulse ready)"
        echo "  File:     $OUTPATH"
        echo "  Samples:  $DATA_LINES"
        echo "  Duration: ~${SECONDS_RECORDED} seconds"
        echo "  Label:    $LABEL"
        echo "============================================"
        echo ""
        echo "  Upload directly to Edge Impulse:"
        echo "  1. Go to studio.edgeimpulse.com"
        echo "  2. Data Acquisition -> Upload Data"
        echo "  3. Drag '$OUTFILE' in"
        echo "  4. Set label to '$LABEL'"
        echo "  5. Click Upload"
        echo "============================================"
    fi
    exit 0
}

trap cleanup SIGINT

echo "Sending start command to watch..."
echo -n "s" > "$PORT"
sleep 0.3

echo "Recording '$LABEL' data... Press Ctrl+C to stop."
echo ""

# Start capture in background
cat "$PORT" > "$OUTPATH" &
CAPTURE_PID=$!

# Timer loop
START_TIME=$(date +%s)
while kill -0 $CAPTURE_PID 2>/dev/null; do
    CURRENT_TIME=$(date +%s)
    ELAPSED=$((CURRENT_TIME - START_TIME))
    printf "\r  Elapsed: %02d:%02d " $((ELAPSED/60)) $((ELAPSED%60))
    sleep 1
done
