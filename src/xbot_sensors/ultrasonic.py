
class Ultrasonic_Sensors:

    def __init__(self, side='ALL'):
        Ultrasonic = get_robot_cfg()['Sensors']['Ultrasonic']
        self.GPIO_TRIGGER = Ultrasonic['Trigger']
        self.GPIO_ECHO_F = Ultrasonic['Front']
        self.GPIO_ECHO_B = Ultrasonic['Back']
        self.GPIO_ECHO_R = Ultrasonic['Right']
        self.GPIO_ECHO_L = Ultrasonic['Left']
        self.MAX_RANGE = Ultrasonic['MaxRange']
        self.f_us = 0
        self.b_us = 0
        self.r_us = 0
        self.l_us = 0
        self.ACCURACY = 2
        self.init_us()
        self.req_side = side
        self.FRONT = 'FRONT'
        self.BACK = 'BACK'
        self.RIGHT = 'RIGHT'
        self.LEFT = 'LEFT'
        self.ALL = 'ALL'

    def init_us(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO_F, GPIO.IN)
        GPIO.setup(self.GPIO_ECHO_B, GPIO.IN)
        GPIO.setup(self.GPIO_ECHO_R, GPIO.IN)
        GPIO.setup(self.GPIO_ECHO_L, GPIO.IN)

    def distance(self, side):
        GPIO.output(self.GPIO_TRIGGER, True)
        time.sleep(1e-05)
        GPIO.output(self.GPIO_TRIGGER, False)
        StartTime = time.time()
        StopTime = time.time()
        while GPIO.input(side) == 0:
            StartTime = time.time()

        while GPIO.input(side) == 1:
            StopTime = time.time()

        TimeElapsed = StopTime - StartTime
        distance = TimeElapsed * 17150
        if distance > self.MAX_RANGE:
            return self.MAX_RANGE
        else:
            return distance

    def measure_dist(self):
        self.r_us = self.distance(self.GPIO_ECHO_R)

    def read_us(self):
        self.measure_dist()
        front_us = round(self.f_us, self.ACCURACY)
        back_us = round(self.b_us, self.ACCURACY)
        right_us = round(self.r_us, self.ACCURACY)
        left_us = round(self.l_us, self.ACCURACY)
        if self.req_side == self.FRONT:
            return front_us
        if self.req_side == self.BACK:
            return back_us
        if self.req_side == self.RIGHT:
            return right_us
        if self.req_side == self.LEFT:
            return left_us
        if self.req_side == self.ALL:
            return (front_us, back_us, left_us, right_us)
