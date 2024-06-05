import serial
import time
import redis

serialPort = serial.Serial(
    port="COM6", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
)
serialString = ""  # Used to hold data coming over UART
r = redis.Redis(
    host='localhost',
    port=6379,
    decode_responses=True)
flagState = False
lastFlagState = False
while 1:
    # Read data out of the buffer until a carraige return / new line is found
    serialString = serialPort.readline()

    # Print the contents of the serial data
    try:
        serialString = serialString.decode("Ascii");
        if (serialString == "23\r\n"):
            flagState = True
            if (flagState != lastFlagState):
                print("True")
                r.publish("button", str("True"))
            lastFlagState = flagState
           
        else:
            flagState = False
            if (flagState != lastFlagState):
                print("False")
                r.publish("button", str("False"))
            lastFlagState = flagState
        # print(serialString.decode("Ascii"))
    except:
        print("ERROR")