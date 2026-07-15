import serial
import serial.tools.list_ports
import time

def scan_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    print(f"Scanning ports: {ports}")
    
    for p in ports:
        try:
            ser = serial.Serial(p, 115200, timeout=1)
            print(f"Listening on {p} for 2 seconds...")
            start = time.time()
            data = bytearray()
            while time.time() - start < 2.5:
                chunk = ser.read(1024)
                if chunk:
                    data.extend(chunk)
            
            ser.close()
            if len(data) > 0:
                print(f"!!! Received {len(data)} bytes on {p} !!!")
                if b'LOG:' in data:
                    print(f"Found LOG string on {p}!")
                if b'\x00\x00\x80\x7F' in data:
                    print(f"Found VOFA JustFloat tail on {p}!")
        except Exception as e:
            pass

if __name__ == '__main__':
    scan_ports()
