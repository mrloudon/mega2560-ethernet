#!/usr/bin/python3

import socket

UDP_IP = "192.168.20.255"
UDP_PORT = 8888
MESSAGE = b"Got it!"


sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                     
sock.bind((UDP_IP, UDP_PORT))

print("Reciever")

while True:
	data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
	print("received message:", data)
	print("from:", addr)
	sock.sendto(MESSAGE, addr)