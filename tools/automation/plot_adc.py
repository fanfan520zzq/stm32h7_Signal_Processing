import serial
import time
import sys
import argparse
import csv
import matplotlib.pyplot as plt
import matplotlib
import numpy as np

matplotlib.rcParams['font.sans-serif'] = ['SimHei'] # Support Chinese titles if needed
matplotlib.rcParams['axes.unicode_minus'] = False

def run_adc_test(port, baud, length):
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

    print(f"Sending CMD:ADC_TEST,{length}...")
    ser.write(f"CMD:ADC_TEST,{length}\r\n".encode('utf-8'))

    start_time = time.time()
    recording = False
    data_ch1 = []
    data_ch2 = []
    
    # Wait up to 5 seconds for response and data
    while time.time() - start_time < 5.0:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            
            if line.startswith("ADC_DATA_START"):
                print(f"Recording ADC data... ({line})")
                recording = True
                continue
            elif line == "ADC_DATA_END":
                print("Finished recording ADC data.")
                break
                
            if recording:
                try:
                    v1, v2 = line.split(',')
                    data_ch1.append(int(v1))
                    data_ch2.append(int(v2))
                except ValueError:
                    pass
        else:
            time.sleep(0.005)
            
    ser.close()

    if not data_ch1:
        print("Error: No data received.")
        sys.exit(1)
        
    print(f"Received {len(data_ch1)} points.")

    # Save to CSV
    csv_file = 'adc_data.csv'
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['CH1_RAW', 'CH2_RAW'])
        for v1, v2 in zip(data_ch1, data_ch2):
            writer.writerow([v1, v2])
    print(f"Data saved to {csv_file}")

    # Process Data
    # STM32H7 ADC is 16-bit, Reference voltage usually 3.3V
    v_ref = 3.3
    max_val = 65535.0
    fs = 1024000.0 # 1.024 MHz

    ch1_v = np.array(data_ch1) / max_val * v_ref
    ch2_v = np.array(data_ch2) / max_val * v_ref

    t = np.arange(len(ch1_v)) / fs * 1000 # time in ms

    vpp1 = np.max(ch1_v) - np.min(ch1_v)
    bias1 = (np.max(ch1_v) + np.min(ch1_v)) / 2
    
    vpp2 = np.max(ch2_v) - np.min(ch2_v)
    bias2 = (np.max(ch2_v) + np.min(ch2_v)) / 2

    print(f"CH1 (ADC2_PC4): Vpp={vpp1:.3f}V, Bias={bias1:.3f}V")
    print(f"CH2 (ADC1_PB1): Vpp={vpp2:.3f}V, Bias={bias2:.3f}V")

    # Plot
    plt.figure(figsize=(10, 6))
    
    plt.subplot(2, 1, 1)
    plt.plot(t, ch1_v, marker='.', linestyle='-', color='b', alpha=0.7, markersize=4, label='ADC2_PC4 (CH1)')
    plt.title(f'ADC2 (PC4) - Fs=1.024MHz | Vpp={vpp1:.2f}V | Bias={bias1:.2f}V')
    plt.ylabel('Voltage (V)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    plt.subplot(2, 1, 2)
    plt.plot(t, ch2_v, marker='.', linestyle='-', color='r', alpha=0.7, markersize=4, label='ADC1_PB1 (CH2)')
    plt.title(f'ADC1 (PB1) - Fs=1.024MHz | Vpp={vpp2:.2f}V | Bias={bias2:.2f}V')
    plt.xlabel('Time (ms)')
    plt.ylabel('Voltage (V)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    plt.tight_layout()
    # plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM16", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--length", type=int, default=1024, help="Sampling length")
    args = parser.parse_args()
    
    run_adc_test(args.port, args.baud, args.length)
