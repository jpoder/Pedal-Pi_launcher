
import RPi.GPIO as GPIO
import threading
from time import sleep
import subprocess
import time

import Adafruit_SSD1306			#Use this one if you are using the OLED display

from PIL import Image



RST = 24						# the SSD1306 driver requires a reset line, but the hardware doesn't support.  whatever!

BtnPin = 1  					#button on the rotary encoder

Enc_A = 4  						# Encoder input A: input GPIO 4 
Enc_B = 23  			        # Encoder input B: input GPIO 23 

Rotary_counter = 0  			# Start counting from 0
Current_A = 1					# Assume that rotary switch is not 
Current_B = 1					# moving while we init software

LockRotary = threading.Lock()		# create lock for rotary switch

curr_active = False
launchPedal = False

# initialize interrupt handlers
def init():
	global disp, effects, numPedals, curr_active, launchPedal
	effects = ['Reverb','Booster','Distortion','Echo','Looper','Fuzz','Tremolo','Octaver','Bitcrusher']	
		
	numPedals = len(effects)
#	print numPedals	
	
	# init interrupts
	GPIO.setwarnings(True)
	GPIO.setmode(GPIO.BCM)					# Use BCM mode
											# define the Encoder switch inputs
	GPIO.setup(Enc_A, GPIO.IN) 				
	GPIO.setup(Enc_B, GPIO.IN)
											# setup callback thread for the A and B encoder 
											# use interrupts for all inputs
	GPIO.add_event_detect(Enc_A, GPIO.RISING, callback=rotary_interrupt) 				
	GPIO.add_event_detect(Enc_B, GPIO.RISING, callback=rotary_interrupt) 				

	#setup the button on the rotary encoder
	GPIO.setup(BtnPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)    # Set BtnPin's mode is input, and pull up to high level(3.3V)
	GPIO.add_event_detect(BtnPin, GPIO.RISING, callback=BtnSelect, bouncetime=200)

	#init display
	# 128x64 display with hardware I2C:
	disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST)   #not sure why this driver requires a reset pin...the hardware doesn't have one.  meh.

	
	# Initialize library.
	disp.begin()

	# Clear display.
	disp.clear()
	disp.display()


# set the starting image
# init the selected values to 0

	# load an image, resize it, and convert to 1 bit color.

	image = Image.open('./images/Cole.jpg').resize((128,64), Image.ANTIALIAS).convert('1')

	disp.image(image)
	disp.display()
	curr_active = False
	return

# Rotarty encoder interrupt:
# this one is called for both inputs from rotary switch (A and B)
def rotary_interrupt(A_or_B):
	global Rotary_counter, Current_A, Current_B, LockRotary
													# read both of the switches
	Switch_A = GPIO.input(Enc_A)
	Switch_B = GPIO.input(Enc_B)
													# now check if state of A or B has changed
													# if not that means that bouncing caused it
	if Current_A == Switch_A and Current_B == Switch_B:		# Same interrupt as before (Bouncing)?
		return										# ignore interrupt!

	Current_A = Switch_A								# remember new state
	Current_B = Switch_B								# for next bouncing check


	if (Switch_A and Switch_B):						# Both one active? Yes -> end of sequence
		LockRotary.acquire()						# get lock 
		if A_or_B == Enc_B:							# Turning direction depends on 
			Rotary_counter += 1						# which input gave last interrupt
		else:										# so depending on direction either
			Rotary_counter -= 1						# increase or decrease counter
		LockRotary.release()						# and release lock
	return											# THAT'S IT

#Button interrupt for selector
def BtnSelect(whichButton):
	global curr_active,launchPedal


	curr_active = not curr_active                 # button was pressed so toggle the variable to activate/deactivate it 
	if curr_active == True:
		# print("activate pedal")		
	else:
		# print("deactivate pedal")
		
	launchPedal = True								#notification to the main loop that we are changing the pedal state

	return curr_active,launchPedal

def switch_image(argument):							#return the path to the image needed
    switcher = {
        0: "./images/Reverb_logo.jpg",
        1: "./images/Booster_logo.png",
        2: "./images/Distortion_logo.png",
        3: "./images/echo_logo.jpg",
        4: "./images/looper_logo.jpg",
        5: "./images/Fuzz_logo.png",
        6: "./images/Tremolo_logo.png",
        7: "./images/Octave_logo.png",
        8: "./images/Bitcrusher_logo.png"
    }
    return switcher.get(argument, "./images/Cole.jpg")


# Main loop. Check to see if the rotary encoder has changed position, update the image displayed
# check to see if the pedal needs to be activated/deactivated (subprocess.Popen or subprocess.kill)
# this calls or kills the c routine with the effect in a separate process and keeps running this main loop.
# clean-up the GPIO if you exit this program with keyboard break or exception.
def main():
	global Rotary_counter, LockRotary, disp, curr_active, launchPedal
	

	Image_cnt = 0									# Current Volume	
	NewCounter = 0								# for faster reading with locks
						

	init()										# Init interrupts, GPIO, ...
				
	try:
		 while True :								# start test 
			sleep(0.1)								# sleep 100 msec
		
												# because of threading make sure no thread
												# changes value until we get them
												# and reset them
			if (curr_active == False):					# only look for rotating if the pedal is not active.					
				LockRotary.acquire()					# get lock for rotary switch
				NewCounter = Rotary_counter			# get counter value
				Rotary_counter = 0						# RESET IT TO 0
				LockRotary.release()					# and release lock
						
				if (NewCounter !=0):					# Counter has CHANGED
					Image_cnt = (Image_cnt + NewCounter*abs(NewCounter))	# Decrease or increase volume 
					if Image_cnt < 0:						# limit the values to 0...number of pedals
						Image_cnt = numPedals + Image_cnt   #wraparound
					if Image_cnt >= numPedals:					
						Image_cnt = Image_cnt - numPedals
					print NewCounter, Image_cnt			# some test print		
					if ((Image_cnt < 0)  or (Image_cnt > numPedals)):	#sometimes things get confused if you spin the knob fast.  reset to 0 image
						Image_cnt = 0
					image = Image.open(switch_image(Image_cnt)).resize((128, 64), Image.ANTIALIAS).convert('1')
					#print('updating display')
					disp.image(image)
					disp.display()


			if (launchPedal==True):
				if (curr_active == True):
					#//////////////START THE EFFECTS///////////////////////
					if (effects[Image_cnt] == 'Reverb'):
						#print ('Activate Reverb')
						activePedal  = subprocess.Popen("./Pedal-Pi-All-Effects/reverb")
						launchPedal = False
					if (effects[Image_cnt] == 'Booster'):
						#print ('Activate Booster')
						activePedal = subprocess.Popen("./Pedal-Pi-All-Effects/booster")
						launchPedal = False

					if (effects[Image_cnt] == 'Distortion'):
						#print ('Activate Distortion')
						activePedal = subprocess.Popen("./Pedal-Pi-All-Effects/distortion")
						launchPedal = False

					if (effects[Image_cnt] == 'Echo'):
						#print ('Activate Echo')
						activePedal = subprocess.Popen("./Pedal-Pi-All-Effects/echo")
						launchPedal = False

					if (effects[Image_cnt] == 'Looper'):
						#print ('Activate Looper')
						activePedal = subprocess.Popen("./Pedal-Pi-All-Effects/looper")
						launchPedal = False

					if (effects[Image_cnt] == 'Fuzz'):
						#print ('Activate Fuzz')
						activePedal = subprocess.Popen("./Pedal-Pi-All-Effects/fuzz")
						launchPedal = False

					if (effects[Image_cnt] == 'Tremolo'):
						#print ('Activate Tremolo')
						activePedal = subprocess.Popen("./Pedal-Pi-All-Effects/tremolo")
						launchPedal = False

					if (effects[Image_cnt] == 'Bitcrusher'):
						#print ('Activate Bitcrusher')
						activePedal = subprocess.Popen("./Pedal-Pi-All-Effects/bitcrusher")
						launchPedal = False
				else:
					#print('Ending the active Pedal')
					activePedal.kill()
					launchPedal = False
        
		
	except KeyboardInterrupt:
    		print("Program ended by user request")
	except Exception as ex:
    		print("Program ended with error: %s" % (ex.message))
	finally:
    		GPIO.cleanup()		



# start main function
main()


