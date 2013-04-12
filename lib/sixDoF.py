import time, datetime
from math import pi,sin,cos

# -*- coding: utf-8 *-*
class Rotation:
	def __init__(self, wiimote, cwiid):
		self.wiimote = wiimote
		self.wiimote_angle = [0.0, 0.0, 0.0]
		self.object_angle = [0.0, 0.0, 0.0]
		self.last_call = datetime.datetime.now()
		self.inaccuracy = 1e-3
		self.max_len_history = 3##>1, if we want to have more time-dependent values ... maybe for later corrrections
	   # self.len_history = 1
	 #   self.reached_max_len_history = False
		self.angle_rate = []
		self.corrF_rate = [0,0,0]
		self.corrF_angle = [0.1585667954964998, 0.15318022095511635, 0.1557719762138915]

		self.delta_t = []
		self.last_call = datetime.datetime.now()
		for i in range(self.max_len_history):
			self.angle_rate.append([0,0,0])
			self.delta_t.append(0)


	def calibrateAngleRate(self):
		'''get reference-values to correct the angle-rate by meaning(x-values)'''
		self.corrF_rate = [0,0,0]
		n_cal = 1000
		print "calibrating ..."
		raw_input("put your wiimote on a table - still - and press ENTER")
		for n in range(n_cal):
			angle_rate = self.wiimote.state["motionplus"]["angle_rate"]
			self.corrF_rate[0] += angle_rate[0]
			self.corrF_rate[1] += angle_rate[1]
			self.corrF_rate[2] += angle_rate[2]
		self.corrF_rate[0] /= float(n_cal)
		self.corrF_rate[1] /= float(n_cal)
		self.corrF_rate[2] /= float(n_cal)
		print "done\.n"
		time.sleep(0.5)


	def getAngleRate(self, angle_rate_0):
		self.angle_rate.append([0,0,0])
		for i in range(3):
			##correct angle-acceleration (via wiimote-gyroscope) with the calibrated values
			self.angle_rate[-1][i] = (angle_rate_0[i]/self.corrF_rate[i])-1
		self.angle_rate.pop(0)

		##get computer-time to calculate the time-difference delta_t to the last call
		recent_call = datetime.datetime.now()
		self.delta_t.append((self.last_call - recent_call).total_seconds())
		self.delta_t.pop(0)
		self.last_call = recent_call##the new value becomes the old values of the following cal

	def getWiimoteAngle(self):
		for i in range(3): ## add a*dt and correct with the correcture-factor of the calibration
			self.wiimote_angle[i] += (self.angle_rate[-1][i]*self.delta_t[-1])/ self.corrF_angle[i]

	def getObjectAngle(self):
		for i in range(3): ## add a*dt and correct with the correcture-factor of the calibration
			self.object_angle[i] += (self.angle_rate[-1][i]*self.delta_t[-1])/ self.corrF_angle[i]

	def smoothenAngleRate(self):
		NotImplemented

		 #   ##filter random noise
		#   if abs(self.angle_rate[i]) < self.inaccuracy:
		 #	   self.angle_rate[i] = 0

	def calibrateAngle(self):
		self.corrF_angle = [1,1,1]
		n_cal_angle = 5
		print "LIFT your arm (press B) UP to 180degree and press (A)"
		print "do this for %s times" %n_cal_angle
		self.calibrate_getMinMaxAngle(0, n_cal_angle)
		print "now TILT your wiimote from zero-position 180degree FROM LEFT (press B) TO RIGHT to and press (A)"
		print "do this for %s times" %n_cal_angle
		self.calibrate_getMinMaxAngle(1, n_cal_angle)
		print "now MOVE your arm FROM THE LEFT (press B) 180degree TO THE RIGHT and press (A)"
		print "do this for %s times" %n_cal_angle
		self.calibrate_getMinMaxAngle(2, n_cal_angle)

	def calibrate_getMinMaxAngle(self,dim, n_cal_angle):
		got_zero_pos = False
		i = 0
		while True:
			wiimote_state = self.wiimote.state
			self.getAngleRate(list(wiimote_state["motionplus"]["angle_rate"]))
			self.getWiimoteAngle()
			if self.wiimote.state["buttons"] == self.cwiid.BTN_B:
				time.sleep(0.5)
				min_angle = self.wiimote_angle[dim]
				got_zero_pos = True
			if self.wiimote.state["buttons"] == self.cwiid.BTN_A:
				if not got_zero_pos:
					print "go to position zero and press (B) first"
					time.sleep(0.5)
					continue
				time.sleep(0.5)
				max_angle = self.wiimote_angle[dim]
				self.corrF_angle[0] += (max_angle - min_angle)
				i+=1
				print "got %s'st value" %i
				got_zero_pos = False
				if i == n_cal_angle:
					self.corrF_angle[dim] /= (i * pi)
					break
				time.sleep(0.5)
				print "continue"
		print "done\.n"
	 #   time.sleep(0.5)


class Translation:
	def __init__(self):
		self.corrF_acc1 = [127.0, 128.0, 128.0]
		self.corrF_acc2 = [24.0, 24.0,24.0]
		self.g = 9.81#GRAVITATION
		self.g_vector = [0.0 ,0.0 ,self.g]
		#self.corrF_acc2 = [corrF_acc2
		self.max_len_history = 3
		self.acc = []#acceleration
		self.delta_t = []
		self.last_call = datetime.datetime.now()
		self.position = [0.0, 0.0, 0.0]

		for i in range(self.max_len_history):
			self.acc.append(self.g_vector)
			self.delta_t.append(0)

	def getAcc(self, acc_0):
		self.acc.append([0,0,0])#self.angle[-1])##get last angle
		for i in range(3):
			self.acc[-1][i] = ((acc_0[i] - self.corrF_acc1[i]) / self.corrF_acc2[i]) * self.g

		self.acc.pop(0)#remove the fist (oldest) value

		##get computer-time to calculate the time-difference delta_t to the last call
		recent_call = datetime.datetime.now()
		self.delta_t.append((self.last_call - recent_call).total_seconds())
		self.delta_t.pop(0)
		self.last_call = recent_call##the new value becomes the old values of the following call


	def correctAccByAngle(self):
		self.acc[-1][2] -= self.g

  #  def smoothAcceleration(self):
##		if abs(acc[0]) < inaccuracy:
##			acc[0] = 0
##		acc[1] *= acc_g
##		if abs(acc[1]) < inaccuracy:
##			acc[1] = 0
##		acc[2] *= acc_g
##		if abs(acc[2]) < inaccuracy:
##			acc[2] = 0


	def getPosition(self):#position_time_1,inaccuracy,acc, position):
		for i in range(3):
			self.position[i] += (0.5*self.acc[-1][i])*(self.delta_t[-1]**2)




	def accSmooth(acc, new_acc):
	##		if abs(acc_corr[0]-new_acc_corr[0])>0.001:
	##		print new_acc_corr[0], acc_corr[0],acc[0]
	##		#new_acc_corr[0] = acc_corr[0]
	##		#acc_corr[0] = (acc_corr[0]+new_acc_corr[0])/2
	##		#print acc_corr[0], new_acc_corr, (acc_corr[0]+new_acc_corr[0])/2
	##	if abs(new_acc_corr[0]) < 10:# and abs(acc_corr[1]-new_acc_corr[1])<0.01:
	##		acc_corr[1] = (acc_corr[1]+new_acc_corr[1])/2
	##	if abs(new_acc_corr[0]) < 10:# and abs(acc_corr[2]-new_acc_corr[2])<0.01:
	##		acc_corr[2] = (acc_corr[2]+new_acc_corr[2])/2
		acc_smooth =[0,0,0]
		for i in range(3):

			if abs(acc[i]-new_acc[i])<0.7:
				acc_smooth[i] = (new_acc[i] + acc[i])/2
			else:
				acc_smooth[i] = acc[i]
	 #   new_acc = deepcopy(acc)
		return acc_smooth




def transformAngle(moment,angle):
	'''http://de.wikipedia.org/wiki/Eulersche_Winkel'''

	convention = "Luftfahrtnorm"#X#Y

	if convention == "Luftfahrtnorm":

		#psi = angle[2]
		#theta = angle[1]
		#phi = angle[0]

		#psi = 0#angle[2]#nicht 1
		#theta = 0##		 #angle[2]#ok? #nei +1: x und z verk.##nicht -2
		#phi = angle[0]#ok nicht 1

		#rot_matrix = [##Luftfahrtnorm (DIN 9300)##angle[2]->angle[1]->angle[0]
						#[
							#cos(theta)*cos(psi),
							#cos(theta)*sin(psi),
							#-sin(theta)
						#],
						#[
							#sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),
							#sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),
							#sin(phi)*cos(theta)
						#],
						#[
							#cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),
							#cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),
							#cos(phi)*cos(theta)
						#]
					  #]
		psi = -angle[0]#angle[2]#angle[0]#angle[2]#nicht 1
		theta =0 #ok? #nei +1: x und z verk.##nicht -2
		phi = angle[2]#angle[1]#-angle[0]#ok nicht 1
		
		rot_matrix = [##Luftfahrtnorm (DIN 9300) M_RNG

						[
							cos(theta)*cos(phi),
							sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi),
							cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi)
						],
						[
							cos(theta)*sin(phi),
							sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi),
							cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(theta)
						],
						[
							-sin(theta),
							cos(theta)*sin(psi),
							cos(theta)*cos(psi)
						]
					  ]
		#print rot_matrix


	elif convention == "X":
##		psi = angle[1]
##		theta = angle[2]
##		phi = angle[0]

		psi = angle[0]
		theta = angle[1]
		phi = angle[2]

		psi = -angle[1]
		theta = angle[0]
		phi = angle[2]

		psi = angle[1]
		theta = angle[0]
		phi = -angle[2]

		psi = angle[1]#-angle[2]
		theta = -angle[0]#ok
		phi = 0#0#angle[2]#0#angle[0]#0#-angle[2]

		rot_matrix = [
						[
							cos(psi)*cos(phi)-sin(psi)*cos(theta)*sin(phi),
							-cos(psi)*sin(phi)-sin(psi)*cos(theta)*cos(phi),
							sin(psi)*sin(theta)
						],
						[
							sin(psi)*cos(phi)+cos(psi)*cos(theta)*sin(phi),
							cos(psi)*cos(theta)*cos(phi)-sin(psi)*sin(phi),
							-cos(psi)*sin(theta)
						],
						[
							sin(theta)*sin(phi),
							sin(theta)*cos(phi),
							cos(theta)
						]
					  ]
	elif convention == "Y":
		rot_matrix = [
						[
							-sin(psi)*sin(phi)+cos(psi)*cos(theta)*cos(phi),
							-sin(psi)*cos(phi)-cos(psi)*cos(theta)*sin(phi),
							cos(psi)*sin(theta)
						],
						[
							cos(psi)*sin(phi)+sin(psi)*cos(theta)*cos(phi),
							cos(psi)*cos(phi)-sin(psi)*cos(theta)*sin(phi),
							sin(psi)*sin(theta)

						],
						[
							-sin(theta)*cos(phi),
							sin(theta)*sin(phi),
							cos(theta)
						]
					  ]
	else:
		NotImplemented

	moment_corr = [0,0,0]
	##calc matrix-pruduct
	for i in range(3):
		moment_corr[i] = moment[0]*rot_matrix[i][0] + moment[1]*rot_matrix[i][1] + moment[2]*rot_matrix[i][2]
	return moment_corr
