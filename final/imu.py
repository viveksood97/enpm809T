import serial

ser = serial.Serial('/dev/ttyUSB0', 9600)

count = 0

while True:
    if(ser.in_waiting > 0):
        count += 1
        line = ser.readline()

        if(count > 10):

            line = line.rstrip().lstrip()

            line = str(line)
            line = line.strip("'")
            line = line.strip("b'")

            line = float(line)

            print(line,"\n")
    print("hello")
