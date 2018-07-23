# -*- coding: cp936 -*-
import socket
import time
s=socket.socket()
s.bind(('127.0.0.1', 5100)) #192.168.13.20
s.listen(5)
cs,address = s.accept()
print 'got connected from',address

while True:
    cs.send('#,ODOM,-10,-10,10,10,CAR_DATA,123,456,789,012,345,CONTROL,0,10,!fskgjbdsklbg')
    time.sleep(0.05)
ra=cs.recv(512)
print ra
print len(ra)
ra=cs.recv(512)
print ra
print len(ra)

'''
cs.send('0.5,-0.8,0sssss.0')

ra=cs.recv(512)
print ra
print len(ra)
ra=cs.recv(512)
print ra
print len(ra)
'''
cs.close()
