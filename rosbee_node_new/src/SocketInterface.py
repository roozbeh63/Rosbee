import socket

class SocketInterface(object):
    def __init__(self, hostName, portNumber):
        self.socket = socket.socket()         # Create a socket object
        self.host = hostName # Get local machine name
        self.port = portNumber                # Reserve a port for your service.
        self.conntionStatus = False
    def OpenConnection(self):
        try:
            self.socket.connect((self.host, self.port))
            self.conntionStatus = True
        except:
            print ("socket connection is not opening")
            self.conntionStatus = False
    def CloseConnection(self):
        self.socket.close()
        self.conntionStatus = False

    def GetMessage(self):
        if self.conntionStatus:
            return self.socket.recv(1024)
        else:
            return None
