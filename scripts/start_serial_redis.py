import serial
import sys
import redis
BAUD = 115200

if len(sys.argv) == 1:
	print("ERROR: Please enter a name")
	sys.exit()

name = sys.argv[1]
names = {"chase": "/dev/cu.usbserial-0001", "omar": "COM6"}
if not name in names:
	print("ERROR: Please enter one of the following names", names.keys())
port = names[name]
def main():
	r = redis.Redis(
    host='localhost',
    port=6379,
    decode_responses=True)
	
	serPort = serial.Serial(port="COM6", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
	serialString = ""
	while True:	
		#RAW = ser.readline().decode("Ascii")
		decoded = serPort.readline()
		#serialString = serialString.replace("\r\n", "")
		# serialString = serialString.replace("\n", "")
		#serialString = serialString.decode("Ascii")
		try:
			decoded = str(decoded.decode("Ascii"))
			#print(serialString.decode("Ascii"))
		except:
			continue
			print("ERROR")
		decoded = decoded.replace("\r", "")
		decoded = decoded.replace("\n", "")
		# value = int(text)
		print(decoded=="23")
		r.publish("button", str(decoded=="23"))
		#text = RAW.replace("")
		#print(ser.readline())


if __name__ == "__main__":
	main()
