import lgpio
import time
import datetime

# in seconds
TIME_BETWEEN_SCANS = 1.0*10**-3

# TODO having each node make their own of this is lowk a terrible idea
# Open the GPIO chip (usually chip 0 is used on the Raspberry Pi)
chip = lgpio.gpiochip_open(0)

# this sonar class will only send pulses once every milisecond
class Sonar:
    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.time_of_last_reading = datetime.datetime.now()

        # Set the GPIO pins as an output
        lgpio.gpio_claim_output(chip, self.trigger_pin)
        lgpio.gpio_claim_input(chip, self.echo_pin)

    # will always return a float, float will be -1.0 if no measurement could be made
    def single_measure(self):
        # first we are gonna check to see if 
        # enough time has passed between measurements
        delta_t_seconds = (datetime.datetime.now() - self.time_of_last_reading).total_seconds()
        if(delta_t_seconds < TIME_BETWEEN_SCANS):
            print("this should never get called")
            return -1.0

        self.send_pulse()
        resp_in_delta = self.read_response()
        resp_in_seconds = resp_in_delta.total_seconds() * 10**6
        distance_cm = (resp_in_seconds*0.0343) /2

        self.time_of_last_reading = datetime.datetime.now()

        return distance_cm

    # returns a datetime.datetime deltatime or something similar
    def read_response(self):
        while lgpio.gpio_read(chip, self.echo_pin) == 0:
            pass

        before = datetime.datetime.now()

        while lgpio.gpio_read(chip, self.echo_pin) == 1:
            pass

        after = datetime.datetime.now()

        return after-before

    def send_pulse(self):
        lgpio.gpio_write(chip, self.trigger_pin, 1)
        time.sleep(10 * 10**-6)
        lgpio.gpio_write(chip, self.trigger_pin, 0)

if __name__ == "__main__":
    while True:
        send_measure()

    lgpio.gpiochip_close(chip)
