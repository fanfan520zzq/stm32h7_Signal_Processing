import re
import sys
import matplotlib.pyplot as plt
import numpy as np

def parse_log_and_plot(filename):
    freqs = []
    mags = []
    phases = []
    
    # Regex to match the printf format:
    # [pt] f_set=100.0 N=... f_act=100.00 L=... |H|=0.0656 phase=1.70deg ...
    # We want f_act, |H|, and phase.
    pattern = re.compile(r'f_act=([0-9.]+).*?\|H\|=([0-9.]+).*?phase=([-0-9.]+)deg')
    
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"Error: Could not find '{filename}'")
        print("Please copy the serial output into this file and try again.")
        sys.exit(1)
        
    for line in lines:
        match = pattern.search(line)
        if match:
            f_act = float(match.group(1))
            mag = float(match.group(2))
            phase_deg = float(match.group(3))
            
            freqs.append(f_act)
            mags.append(mag)
            phases.append(phase_deg)
            
    if not freqs:
        print(f"No valid data points found in {filename}.")
        sys.exit(1)
        
    print(f"Successfully parsed {len(freqs)} points.")
    
    # Plotting
    freqs = np.array(freqs)
    mags = np.array(mags)
    phases = np.array(phases)
    
    # Convert Magnitude to dB if desired (assuming mag is linear ratio)
    # Using 20*log10(mag) for voltage/amplitude ratio
    # Add a small epsilon to avoid log(0)
    mags_db = 20 * np.log10(mags + 1e-12)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    # Magnitude Plot
    ax1.plot(freqs, mags_db, 'b.-')
    ax1.set_xscale('log')
    ax1.set_ylabel('Magnitude (dB)')
    ax1.set_title('Bode Plot')
    ax1.grid(True, which="both", ls="-", alpha=0.5)
    
    # Phase Plot
    ax2.plot(freqs, phases, 'r.-')
    ax2.set_xscale('log')
    ax2.set_xlabel('Frequency (Hz)')
    ax2.set_ylabel('Phase (degrees)')
    ax2.grid(True, which="both", ls="-", alpha=0.5)
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    log_file = 'serial_log.txt'
    if len(sys.argv) > 1:
        log_file = sys.argv[1]
    else:
        print(f"Usage: python generate_c.py <log_file.txt>")
        print(f"Assuming default log file: {log_file}")
        
    # Create an empty template file if it doesn't exist
    import os
    if not os.path.exists(log_file):
        with open(log_file, 'w') as f:
            f.write("Paste your [pt] f_set=... lines here from the serial console.\n")
        print(f"Created empty {log_file}. Please paste your serial data into it and run again.")
    else:
        parse_log_and_plot(log_file)
