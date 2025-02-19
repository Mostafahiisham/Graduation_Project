import serial

ser = serial.Serial(port='/dev/ttyUSB0',baudrate = 9600)


while True:
    data =input("") 
    value =ser.write(bytes(data,'utf-8'))
