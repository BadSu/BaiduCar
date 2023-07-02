import wiringpi
import time

if wiringpi.wiringPiSetupPhys() == -1:
    print("Init error!") 
wiringpi.pinMode(7, 1)       # Set pin 6 to 1 ( OUTPUT )

while 1:
    wiringpi.digitalWrite(7, 1)  # Write 1 ( HIGH ) to pin 6
    time.sleep(2)
    wiringpi.digitalWrite(7, 0)  # Write 1 ( HIGH ) to pin 6
    time.sleep(2)


