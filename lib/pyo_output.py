##wavegenerator
import pyo# import *
import cwiid

class waveGenerator:
	def __init__(self):
		# Set Up Server
		self.server = pyo.Server(audio="jack")
		#s.setMidiInputDevice(2) # Change as required
		self.server.boot()
		self.server.start()
		self.freq = [0]
	self.sum_freq = 0
		self.mul = [0]#amplitude-multiplier ...volume
		self.wave = pyo.Sine(freq=self.freq, mul=self.mul)
		self.wave.out()
		self.level = 0 #via plus/minus
		self.max_level = 0


		
	def play(self,angle,position,button):

		##set freq in chosen level

		if button != cwiid.BTN_B:

			self.freq[self.level] = angle[0]*(200/(self.level*10+1))
			self.mul[self.level] = angle[1]*(self.level*10+1)
			print self.freq, self.mul
			self.sum_freq = self.freq[0]
			for i in range(self.max_level):
				self.sum_freq += pyo.Sine(freq=self.freq[i+1], mul=self.mul[i+1])
			

		  #  sum_mul = 0
		   # for i in range(self.max_level + 1):
		   #	 sum_mul += self.mul[i]

			
			self.wave.setFreq(self.sum_freq)
			self.wave.setMul(0)

			
		else:
			self.wave.setMul(self.mul[0])
			
		if button == cwiid.BTN_PLUS:
			if self.level == self.max_level:##append new level
				self.freq.append(0)
				self.mul.append(0)
				self.max_level += 1
			self.level += 1
		
		if button == cwiid.BTN_MINUS:
			if self.level > 0:
				self.level -= 1



				
		  #  self.freq += self.wave
		 #   time.sleep(0.3)



		#if button = cwiid.BTN_A:
		#wave.
		
