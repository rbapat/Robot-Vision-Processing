from networktables import NetworkTables
import socket
import cv2
import numpy as np

sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sock.bind(('',5810))
sock.listen(5)
NetworkTables.initialize(server='roboRIO-263-FRC.local')

'''
script that runs on host computer during match. Selects autonomous mode at
beginning and then start receiving camera feed from the client(raspberry pi)
'''
def main():
	modeList = ['Middle With Shot - %d', 'Left Gear Forward - %d', 'Right Gear Forward - %d', 'Middle Gear No Shot - %d', 'Left Gear Still - %d', 'Right Gear Still - %d', 'Nothing - %d']
	for i,e in enumerate(modeList):
		print(e % i)
	choice = int(input('What mode do you wanna do...\n'))
	auto = NetworkTables.getTable('autoData')
	auto.putString('mode', modeList[choice].replace(' - %d',''))
	try:
		input('Sending '  + modeList[choice].replace(' - %d','') + ' via networktables')
	except Exception as e:
		print('you messed up: ' + str(e))

	client, addr = sock.accept()
	while True:
		img = client.recv(300000) #need to fix, make a better protocol for handling sizes
		string = np.fromstring(img,np.uint8)
		im = cv2.imdecode(string, 1)
		try:
			cv2.imshow('Frame',im)
		except Exception as e:
			continue
			
		if cv2.waitKey(10) & 0xFF == 113:
			break
if __name__ == '__main__':
	main()	
