#!/usr/bin/python3
# coding: latin-1

import time
import RPi.GPIO as GPIO

VERSION = ".0.00"
DEBUG = True

usleep = lambda x: time.sleep(x/1000000.0)
micros = lambda: time.time() * 1000000.0


class Stepper :

	class DIRECTION :
		BACKWARD = -1
		FORWARD = 1

	class MOVE_TYPE :
		FULL_STEP=0
		HIGH_TORQUE =1
		HALF_STEP = 2
	
	def __init__(self , nbStep , reductor):
		self.__m_nb_step = int(nbStep * reductor)

		self.__m_inter_delay=0
		self.__m_dir=1
		self.__m_max_freq=100
		self.__m_min_freq=0
		self.__m_freq=0
		self.__m_elaps= micros()
		self.__m_enable_pin=0
		self.__m_nb_pin_A=0
		self.__m_nb_pin_B=0
		self.__m_pin_A=0
		self.__m_pin_B=0
		self.__m_init=False
		self.__m_stop=True
		self.__m_ie=0
		self.__m_stop_step_pos=9

	def get_inter_delay(self):
		return self.__m_inter_delay

	def direction(self , dir = DIRECTION.FORWARD):
		if self.__m_stop :
			if dir >= 0 :
				self.__m_dir = self.DIRECTION.FORWARD 
			else :
				self.__m_dir = self.DIRECTION.BACKWARD

	def max_freq(self , maxFreq):
		self.__m_max_freq = maxFreq

	def min_freq(self , minFreq):
		self.__m_min_freq = minFreq

	def speed_freq(self , hz = float(0.0)):
		self.__m_freq = hz
		
		if self.__m_max_freq < self.__m_freq :
			self.__m_inter_delay = 1000000.0 / float(self.__m_max_freq)
		elif self.__m_min_freq > self.__m_freq :
			self.__m_inter_delay = 1000000.0/ float(self.__m_min_freq)
		else:
			self.__m_inter_delay = 1000000.0/hz


	def speed_rpm(self , rpm = float(0.0)):
		self.__m_inter_delay = 60000000.0/rpm/float(self.__m_nb_step)
		self.__m_freq  =1.0 / (float(self.__m_inter_delay)/1000000.0)

		if self.__m_max_freq < self.__m_freq :
			self.__m_inter_delay = 1000000.0 / float(self.__m_max_freq)
		elif self.__m_min_freq > self.__m_freq :
			self.__m_inter_delay = 1000000.0/ float(self.__m_min_freq)
		else:
			self.__m_inter_delay = self.__m_inter_delay

	def speed_spm(self , spm = float(0.0)):
		self.__m_inter_delay = 60000000.0/spm
		self.__m_freq  =1.0 / (float(self.__m_inter_delay)/1000000.0)

		if self.__m_max_freq < self.__m_freq :
			self.__m_inter_delay = 1000000.0 / float(self.__m_max_freq)
		elif self.__m_min_freq > self.__m_freq :
			self.__m_inter_delay = 1000000.0/ float(self.__m_min_freq)
		else:
			self.__m_inter_delay = self.__m_inter_delay

	def enable(self, enab = False):
		usleep(1)

		if self.__m_enable_pin > 0 :
			if enab :
				GPIO.output(self.__m_enable_pin, GPIO.HIGH)
			else:
				GPIO.output(self.__m_enable_pin, GPIO.LOW)

		usleep(1)

	def start(self):
		if self.__m_stop : 
			self.__m_stop = False
			self.enable(True)
			self.__m_ie = 0
			self.__m_elaps = micros()

	def stop(self):
		if not self.__m_stop :
			self.__m_stop = True
			self.enable(False)
			self.__n_step(9,GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.LOW)
			self.__m_ie = 0
			

	def init(self ,pinA , pinB , pinEnable = 0):
		if DEBUG :
			print("__Stepper.init__")

		if self.__m_stop :
			self.__m_nb_pin_A = len(pinA)
			self.__m_nb_pin_B = len(pinB)

			self.__m_pin_A = pinA
			self.__m_pin_B = pinB

			GPIO.setmode(GPIO.BCM)

			for pin in self.__m_pin_A :
				GPIO.setup(pin, GPIO.OUT , initial=GPIO.LOW)

			for pin in self.__m_pin_B :
				GPIO.setup(pin, GPIO.OUT , initial=GPIO.LOW)

			self.__m_enable_pin = pinEnable
			GPIO.setup(self.__m_enable_pin, GPIO.OUT , initial=GPIO.LOW)

			self.__n_step(9,GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.LOW)

			self.__m_init = True

	def move_laps(self, nbLaps , mvType = MOVE_TYPE.FULL_STEP):
		if DEBUG :
			print("__Stepper.move_laps__")
		self.move_steps(nbLaps * self.__m_nb_step , mvType)

	def move_degs(self, angle , mvType = MOVE_TYPE.FULL_STEP):
		if DEBUG :
			print("__Stepper.move_degs__")
		self.move_steps( int((angle/360.0) * self.__m_nb_step ), mvType)

	def move_steps(self , nbSteps , mvType = MOVE_TYPE.FULL_STEP):
		if DEBUG :
			print("__Stepper.move_steps__")

		if self.__m_init and self.__m_stop :
			self.__m_stop = False
			self.__m_ie = 0
			self.__m_elaps = micros()

			step = int(0)

			self.enable(True)

			relativeStep = self.__m_nb_step

			if mvType == self.MOVE_TYPE.HALF_STEP:
				relativeStep = self.__m_nb_step * 2

			while step < relativeStep :
				relativeInterDelay = self.__m_inter_delay

				if mvType == self.MOVE_TYPE.HALF_STEP :
					relativeInterDelay = self.__m_inter_delay / 2.0

				r_elps = micros()- self.__m_elaps

				if relativeInterDelay <= r_elps - self.__m_ie :
					self.__next_step(mvType)
					self.__m_elaps = micros()
					self.__m_ie = self.__m_inter_delay - r_elps

					step = step + 1

			self.__n_step(9,GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.LOW)
			self.enable(False)

			self.__m_ie = 0
			self.__m_stop = True

	def move_async(self, mvType = MOVE_TYPE.FULL_STEP):
		if DEBUG :
			print("__Stepper.move_async__")
			
		if self.__m_init and self.__m_stop :
			relativeInterDelay = self.__m_inter_delay

			if mvType == self.MOVE_TYPE.HALF_STEP :
				relativeInterDelay = self.__m_inter_delay / 2.0

			r_elps = micros() - self.__m_elaps

			if relativeInterDelay <= r_elps - self.__m_ie :
				self.__next_step(mvType)
				self.__m_elaps = micros()
				self.__m_ie = self.__m_inter_delay - r_elps

		else:
			self.__m_elaps = micros()

	def __next_step(self, mvType = MOVE_TYPE.FULL_STEP):
		next = self.__m_stop_step_pos + self.__m_dir

		if mvType == self.MOVE_TYPE.FULL_STEP :
			if next < 1 :
				next = 4
			elif next > 4:
				next = 1

			if next == 1 :
				self.__n_step(next , GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.LOW)
			elif next == 2 :
				self.__n_step(next , GPIO.LOW,GPIO.LOW,GPIO.HIGH,GPIO.LOW)
			elif next == 3 :
				self.__n_step(next , GPIO.LOW,GPIO.HIGH,GPIO.LOW,GPIO.LOW)
			elif next == 4 :
				self.__n_step(next , GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.HIGH)

		elif mvType == self.MOVE_TYPE.HIGH_TORQUE :
			if next < 1 :
				next = 4
			elif next > 4:
				next = 1

			if next == 1 :
				self.__n_step(next , GPIO.LOW,GPIO.HIGH,GPIO.HIGH,GPIO.LOW)
			elif next == 2 :
				self.__n_step(next , GPIO.LOW,GPIO.HIGH,GPIO.LOW,GPIO.HIGH)
			elif next == 3 :
				self.__n_step(next , GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.HIGH)
			elif next == 4 :
				self.__n_step(next , GPIO.HIGH,GPIO.LOW,GPIO.HIGH,GPIO.LOW)

		elif mvType == self.MOVE_TYPE.HALF_STEP :
			if next < 1 :
				next = 8
			elif next > 8:
				next = 1

			if next == 1 :
				self.__n_step(next , GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.LOW)
			elif next == 2 :
				self.__n_step(next , GPIO.HIGH,GPIO.LOW,GPIO.HIGH,GPIO.LOW)
			elif next == 3 :
				self.__n_step(next , GPIO.LOW,GPIO.LOW,GPIO.HIGH,GPIO.LOW)
			elif next == 4 :
				self.__n_step(next , GPIO.LOW,GPIO.HIGH,GPIO.HIGH,GPIO.LOW)
			elif next == 5 :
				self.__n_step(next , GPIO.LOW,GPIO.HIGH,GPIO.LOW,GPIO.LOW)
			elif next == 6 :
				self.__n_step(next , GPIO.LOW,GPIO.HIGH,GPIO.LOW,GPIO.HIGH)
			elif next == 7 :
				self.__n_step(next , GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.HIGH)
			elif next == 8 :
				self.__n_step(next , GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.HIGH)

	def __n_step(self , idx , a1 , a2 , b1 , b2):
		if self.__m_nb_pin_A > 0 :
			GPIO.output(self.__m_pin_A[0], a1)

			if self.__m_nb_pin_A == 2 :
				GPIO.output(self.__m_pin_A[1], a2)

		if self.__m_nb_pin_B > 0 :
			GPIO.output(self.__m_pin_B[0], b1)

			if self.__m_nb_pin_B == 2 :
				GPIO.output(self.__m_pin_B[1], b2)

		self.__m_stop_step_pos = idx



if __name__ == "__main__" :


	motor = Stepper(48,1)

	motor.max_freq(250)
	motor.min_freq(1)
	motor.speed_rpm(15)
	
	motor.init([5,6],[13,19],0)
	motor.direction(Stepper.DIRECTION.FORWARD)
	motor.move_degs(360.0)

	time.sleep(2)

	motor.direction(Stepper.DIRECTION.BACKWARD)
	motor.move_degs(360.0)