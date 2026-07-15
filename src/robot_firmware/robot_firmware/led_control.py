import serial
import time

# Change port if needed
ser = serial.Serial('/dev/ttyACM2', 9600, timeout=1)

time.sleep(2)  # Wait for Arduino reset

while True:
    cmd = input("Enter 1 (ON), 0 (OFF), q (Quit): ")

    if cmd == 'q':
        break

    ser.write(cmd.encode())

    time.sleep(0.1)

    if ser.in_waiting:
        print(ser.readline().decode().strip())

ser.close()