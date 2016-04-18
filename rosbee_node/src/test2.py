#!/usr/bin/python           # This is server.py file

import socket               # Import socket module

s = socket.socket()         # Create a socket object
host = '145.93.109.109' # Get local machine name
port = 12348               # Reserve a port for your service.
s.bind((host, port))        # Bind to the port
msg = raw_input("->")
s.listen(5)                 # Now wait for client connection.
while True:
   c, addr = s.accept()     # Establish connection with client.
   print 'Got connection from', addr
   print (msg)
   c.send(msg)
   c.close()
