#!/usr/bin/python3
import time, sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

import matplotlib.pyplot as plt

import esp32imu as esp

At = []
Ax = []
Ay = []
Az = []
last_t_us = 0

# how many seconds of samples to keep in the plotting window?
SAMPLE_WINDOW_SEC = 20

def imu_cb(msg):
    global last_t_us
    dt = (msg.t_us - last_t_us) * 1e-6 # us to s
    last_t_us = msg.t_us
    hz = 1./dt
    print('Got IMU at {} us ({:.0f} Hz): {:.2f}, {:.2f}, {:.2f}, \t {:.2f}, {:.2f}, {:.2f}'
            .format(msg.t_us, hz,
                    msg.accel_x, msg.accel_y, msg.accel_z,
                    msg.gyro_x, msg.gyro_y, msg.gyro_z))

    At.append(msg.t_us)
    Ay.append(msg.gyro_y)
    Ax.append(msg.gyro_x)
    Az.append(msg.gyro_z)

    if len(At) > hz*SAMPLE_WINDOW_SEC:
      At.pop(0)
      Ax.pop(0)
      Ay.pop(0)
      Az.pop(0)

def main():
    driver = esp.SerialDriver("/dev/ttyUSB0", 115200)
    time.sleep(0.1)
    driver.registerCallbackIMU(imu_cb)
    # driver.sendRate(1000)

    # https://pyqtgraph.readthedocs.io/en/latest/plotting.html#examples
    pw = pg.plot(title="Accelerometer")
    timer = pg.QtCore.QTimer()
    def update():
        pw.plot(At, Ax, pen=(1,3), clear=True)
        pw.plot(At, Ay, pen=(2,3))
        pw.plot(At, Az, pen=(3,3))
        QtGui.QApplication.processEvents()

    timer.timeout.connect(update)
    timer.start(50) # ms
    QtGui.QApplication.instance().exec_()

    # clean up to prevent error or resource deadlock
    driver.unregisterCallbacks()

    fig, ax = plt.subplots(ncols=3)
    ax[0].hist(Ax, bins=100)
    ax[0].set_title('X')
    ax[0].grid(0.3)
    ax[1].hist(Ay, bins=100)
    ax[1].set_title('X')
    ax[1].grid(0.3)
    ax[2].hist(Az, bins=100)
    ax[2].set_title('X')
    ax[2].grid(0.3)
    plt.show()

if __name__ == '__main__':
    main()