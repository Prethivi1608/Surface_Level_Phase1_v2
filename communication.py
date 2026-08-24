import socket
import serial
import time

class Ethernet():
    def __init__(self):
        self.sock_connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_data(self,ip_details,message):
        data = str(message)
        self.sock_connection.sendto(data.encode(),(ip_details[0],ip_details[1]))

class SerialComm():
    def __init__(self):
        self.rx_data = 0
        self.connection = None

    def connect_to(self,device):
        self.connection = serial.Serial(device[1],device[2], timeout=1)
        print(f"Connection established to {device[0]}")

    def send_data(self,data):
        if self.connection and self.connection.is_open:
            self.connection.write((str(data)+'\n').encode())
            self.connection.flush()

    def send_bytes(self,data):
        if self.connection and self.connection.is_open:
            self.connection.write(data.to_bytes(2, 'little'))
            self.connection.flush()

    def send_byte_packets(self,packets):
        if self.connection and self.connection.is_open:
            self.connection.write(packets)
            self.connection.flush()
    
    def recieve_data(self):
        """Block until a full line is available on the serial port, then return it."""
        while self.connection and self.connection.is_open:
            if self.connection.in_waiting:
                data = self.connection.readline().decode('utf-8', errors='ignore')
                if data:
                    return data
            else:
                time.sleep(0.01)

    def close_port(self):
        if self.connection and self.connection.is_open:
            self.connection.close()
        



                
            