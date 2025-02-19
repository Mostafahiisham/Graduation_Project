import serial 

ser = serial.Serial(port='/dev/ttyUSB0',baudrate=9600)

while True:
    #value = ser.read()
    value = 7
    valueInNumber = int(value)
    print(value)
