#This script reads the data in the serial port and stores it in output.txt
import serial
#Select serial port to read from
serial_port = 'COM3';
baud_rate = 38400; #In arduino, Serial.begin(baud_rate)
#Create or open output file
write_to_file_path = "output.txt";

output_file = open(write_to_file_path, "w+");
ser = serial.Serial(serial_port, baud_rate)
while True:
    line = ser.readline();
    line = line.decode("utf-8") #ser.readline returns a binary, convert to string
    #print(line);
    output_file.write(line);
