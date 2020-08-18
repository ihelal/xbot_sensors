#Libraries
us_side = "RIGHT"
from xbot_sensors.ir import IR as right_ultrasonic
right_us = right_ultrasonic()

if __name__ == '__main__':
    
    while (1):
        if right_us.read_line()[0] == True:
            print("True!")
        elif right_us.read_line()[0] == False:
            print("False!")
        # elif right_us.read_us() < :
        #     print("GG")