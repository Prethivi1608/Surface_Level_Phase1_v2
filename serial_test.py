import serial

PORT = "/dev/ttyUSB0"
BAUD = 115200


ser = serial.Serial(PORT, BAUD, timeout=1)
print(f"Connected to {PORT} @ {BAUD} baud")
print("Waiting for data...\n")

while True:
    try:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line:
            print(line)
    except KeyboardInterrupt:
        print("\nStopping...")
        break
    except Exception as e:
        print(f"Read Error: {e}")

ser.close()