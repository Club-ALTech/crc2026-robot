import serial


ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)


def read_lines_forever(ser):
    """Read lines continuously with error handling"""
    while True:
        try:
            line = ser.readline()
            if line:
                text = line.decode('utf-8', errors='ignore').strip()
                if text:  # Skip empty lines
                    yield text
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Read error: {e}")
            break

# Usage
for line in read_lines_forever(ser):
    print(f"Got: {line}")
    if line == "QUIT":
        break