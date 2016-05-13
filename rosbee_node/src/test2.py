import rbha
import time
from datetime import datetime
rbha.open_serial()
while 1:
    print (datetime.now())
    rbha.send("$11")
    print (rbha.receive())
    print (datetime.now())
