from networktables import NetworkTables

NetworkTables.initialize(server='roboRIO-263-FRC.local')

'''
helps select autonomous mode at the beginning of round
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
		print('you messed up up: ' + str(e))

if __name__ == '__main__':
	main()
