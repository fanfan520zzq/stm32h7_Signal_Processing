import serial
import time
import sys
import json
import argparse

def run_test(port, baud):
    print(f"Connecting to {port} at {baud} baud...")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"Failed to open port {port}: {e}")
        sys.exit(1)

    # Clear buffers
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print("Sending CMD:PING...")
    ser.write(b"CMD:PING\r\n")

    start_time = time.time()
    response_received = False
    logs = []
    
    # Wait up to 3 seconds for response
    while time.time() - start_time < 3.0:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            logs.append(line)
            print(f"RX: {line}")
            if line == "ACK:PONG":
                response_received = True
                break
        time.sleep(0.01)
        
    ser.close()

    result = {
        "status": "PASS" if response_received else "FAIL",
        "logs": logs
    }
    
    print("\n--- Test Result ---")
    print(json.dumps(result, indent=2))
    
    if not response_received:
        sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM16", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()
    
    run_test(args.port, args.baud)
