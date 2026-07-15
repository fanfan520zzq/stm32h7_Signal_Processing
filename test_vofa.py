import serial
import struct
import time

def main():
    print("Opening COM16...")
    try:
        ser = serial.Serial('COM16', 115200, timeout=1)
    except Exception as e:
        print(f"Failed to open COM16: {e}")
        return

    buf = bytearray()
    frame_len = 12 # 2 floats (8 bytes) + 4 bytes tail
    tail = b'\x00\x00\x80\x7F'
    
    print("Listening for VOFA JustFloat frames...")
    
    start_time = time.time()
    frames_received = 0
    
    while time.time() - start_time < 10: # Run for 10 seconds
        data = ser.read(1024)
        if data:
            print(f"RAW DATA: {data}")
            buf.extend(data)
            
            while len(buf) >= frame_len:
                # Search for tail
                tail_idx = buf.find(tail)
                if tail_idx == -1:
                    # Keep the last 3 bytes just in case tail is split
                    if len(buf) > 3:
                        buf = buf[-3:]
                    break
                else:
                    if tail_idx >= 8:
                        # Extract 2 floats before the tail
                        floats_data = buf[tail_idx-8:tail_idx]
                        if len(floats_data) == 8:
                            try:
                                f1, f2 = struct.unpack('<ff', floats_data)
                                frames_received += 1
                                if frames_received % 100 == 0:
                                    print(f"Frame {frames_received}: f1={f1:.3f}V, f2={f2:.3f}V")
                            except Exception as e:
                                pass
                    # Consume buffer up to tail
                    buf = buf[tail_idx+4:]
    
    ser.close()
    print(f"Total frames received: {frames_received}")

if __name__ == '__main__':
    main()
