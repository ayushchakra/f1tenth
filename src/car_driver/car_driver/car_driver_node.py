from adafruit_servokit import ServoKit


def main():
    print('Hi from car_driver.')
    myKit=ServoKit(channels=16)
    myKit.servo[0].angle=90


if __name__ == '__main__':
    main()
