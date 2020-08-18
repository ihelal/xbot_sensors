#!/usr/bin/python3
import rospy
from neopixel import *
import time

class LED():
    def __init__(self):
        # LED strip configuration:
        self.LED_COUNT      = 24      # Number of LED pixels.
        self.LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
        #LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
        self.LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
        self.LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
        self.LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
        self.LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
        self.LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53


        # # Process arguments
        # self.parser = argparse.ArgumentParser()
        # self.parser.add_argument('-c', '--clear', action='store_true', help='clear the display on exit')
        # self.args = self.parser.parse_args()

        # Create NeoPixel object with appropriate configuration.
        self.strip = Adafruit_NeoPixel(self.LED_COUNT, self.LED_PIN, self.LED_FREQ_HZ, self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, self.LED_CHANNEL)
        # Intialize the library (must be called once before other functions).
        self.strip.begin()
        print("gg")
        time.sleep(1)
        print("gg1")

        rospy.init_node('xbot_led', anonymous=False)
        rospy.Subscriber("/chatter", String, led_callback)

    # Define functions which animate LEDs in various ways.
    def colorWipe(self,strip, color, wait_ms=50):
        """Wipe color across display a pixel at a time."""
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, color)
            strip.show()
            time.sleep(100/1000.0)
            
    def led_callback(self,data):
        print(data)
    
    def spin(self):
        self.rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.rate.sleep()
        # self.colorWipe(self.strip,Color(0,0,0))
        # while True:
        #     self.colorWipe(self.strip,Color(0,60,0))
            #self.colorWipe(self.strip,Color(50,0,0))


if __name__ == '__main__':
    run = LED()
    run.spin()
    
