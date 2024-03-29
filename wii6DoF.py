#!/usr/bin/python
import cwiid
import sys
import time
#from sys import stdout, stderr#for statusBar
from copy import deepcopy

##in /lib
from lib.wii_test import wiiTest, callback


import wii6DoFDict as mc

if "block3D" in mc.output:
	from lib.x_output import block3D
if "waveGenerator" in mc.output:
	from lib.pyo_output import waveGenerator
if "playNote" in mc.output:
	from lib.midi_output import playNote

from lib.sixDoF import Rotation
from lib.sixDoF import Translation
from lib.sixDoF import transformAngle

###############################################


def main():
	#Connect to address given on command-line, if present
	raw_input('Put Wiimote in discoverable mode (press 1+2) and press any key to continue')
	#global wiimote
	if len(sys.argv) > 1:
		print "try to connect to given-mac-adr. %s" %sys.argv[1]
		wiimote = cwiid.Wiimote(sys.argv[1])
	else:
		mac_adress = "02:01:54:85:52:61"
		print "try to connect to standart-mac-adr. %s" %mac_adress
		wiimote = cwiid.Wiimote(mac_adress)


	##let wiimote rumble to confirm login
 #   wiimote.rumble = 1
 #   time.sleep(0.2)
 #   wiimote.rumble = 0


	###prepare wiimote
	##get motionplus
	wiimote.enable(cwiid.FLAG_MOTIONPLUS)
	time.sleep(0.5)
	#get button, acc, motionplus und nunchuck
	wiimote.rpt_mode = cwiid.RPT_BTN | cwiid.RPT_ACC | cwiid.RPT_MOTIONPLUS# | cwiid.RPT_NUNCHUK
	time.sleep(0.5)
	wiimote.mesg_callback = callback

	##create instances for translation and rotation
	a = Rotation(wiimote, cwiid)
	p = Translation()

	if "waveGenerator" in mc.output:
		w = waveGenerator()

	a.calibrateAngleRate()

	##choose further way
	print "press (A) to calibrate the angle and (B) to continue"
	while True:
		if wiimote.state["buttons"] == cwiid.BTN_A:
			a.calibrateAngle()
			break
		elif wiimote.state["buttons"] == cwiid.BTN_B:
			#wiiTest(wiimote)
			break


	print "press (B) to print 6Dof, (A) to reset the position the your wiimote and (2) to quit"

	plus_minus_btn = 1

	while True:
		wiimote_state = wiimote.state

		a.getAngleRate(list(wiimote_state["motionplus"]["angle_rate"]))
		a.angle_rate[-1] = transformAngle(a.angle_rate[-1],a.wiimote_angle)
		a.getWiimoteAngle()

		p.getAcc(wiimote_state["acc"])
		#p.correctAccByAngle()
		#p.getPosition()


		if wiimote_state["buttons"] == cwiid.BTN_B:
			a.getObjectAngle()

			if "block3D" in mc.output:
				#block3D(a.object_angle, p.position, loop=False).run()
				block3D(a.wiimote_angle, p.position, loop=False).run()
				#block3D([0,0,0], p.position, loop=False).run()

			if "playMidi" in mc.output:
				playNote(a.wiimote_angle, p.position, plus_minus_btn)
			  #  time.sleep(0.1)

		if "waveGenerator" in mc.output:
			w.play(a.wiimote_angle, p.position, wiimote_state["buttons"])


		   # time.sleep(0.5)

		if wiimote_state["buttons"] == cwiid.BTN_A:
			a.wiimote_angle = [0,0,0]
		 #   a.object_angle = [0,0,0]
			p.position = [0,0,0]

		if wiimote_state["buttons"] == cwiid.BTN_PLUS:
			plus_minus_btn += 1
			time.sleep(0.5)
		if wiimote_state["buttons"] == cwiid.BTN_MINUS:
			plus_minus_btn -= 1
			time.sleep(0.5)
		if wiimote_state["buttons"] == cwiid.BTN_2:
			break

	  #  sys.stdout.write("\r%s  %s  %s  |  %s  %s  %s" %(angle[0],angle[1],angle[2],acc[0],acc[1],acc[2]))
	#	sys.stdout.flush()


	wiimote.close()








################start by:
main()
