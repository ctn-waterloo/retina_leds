import serial
import socket
import time
import thread
import numpy as np

use_serial = False

class Retina(object):
    def __init__(self, port='/dev/ttyUSB0'):
        if use_serial:
            self.serial = serial.Serial(port, baudrate=8000000, rtscts=True)
            time.sleep(0.5)
            self.serial.write("E+\nZ+\n")
            time.sleep(0.5)
            self.serial.write("E+\nZ+\n")
        else:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect(('ctndroid.uwaterloo.ca', 56043))
            time.sleep(0.5)
            self.socket.send('!E1\nE+\n')
            time.sleep(0.5)
            self.socket.send('!E1\nE+\n')

        self.image = np.zeros((128, 128), dtype=float)
        self.last_on = np.zeros((128, 128), dtype=float) + time.time()
        self.last_off = np.zeros((128, 128), dtype=float) + time.time()
        self.p_x = 64.0
        self.p_y = 64.0
        self.deltas = []
        thread.start_new_thread(self.update_loop, ())

    def clear_image(self):
        self.image *= 0.5
        del self.deltas[:]

    def update_loop(self):
        events = 0
        while True:
            if use_serial:
                byte0 = ord(self.serial.read())
            else:
                byte_data = self.socket.recv(4)
                byte0 = ord(byte_data[0])
            if byte0 & 0x80 == 0:
                byte_data = self.socket.recv(4096)
                print 'flush'
                continue
            byte0 = byte0 & 0x7F   # strip the top byte
            if use_serial:
                byte1 = ord(self.serial.read())
            else:
                byte1 = ord(byte_data[1])
            sign = byte1 >= 0x7F
            byte1 = byte1 & 0x7F   # strip the top byte
            self.image[byte1, byte0] += 1 if sign else -1

            if use_serial:
                now = time.time()
            else:
                now = time.time()
                #byte2 = ord(byte_data[2])
                #byte3 = ord(byte_data[3])
                #now = ((byte2 << 8) + byte3) * 0.000001
            if sign:
                delta = now - self.last_off[byte1, byte0]
                self.last_on[byte1, byte0] = now
            else:
                delta = now - self.last_on[byte1, byte0]
                self.last_off[byte1, byte0] = now
            if delta > 0:
                self.deltas.append(delta)

            eta = 0.2
            sigma_t = 0.0005
            t_exp = 0.025

            t_diff = delta - t_exp

            w_t = np.exp(-(t_diff**2)/(2*sigma_t**2))

            r = eta*w_t

            self.p_x = (1-r)*self.p_x + r*byte1
            self.p_y = (1-r)*self.p_y + r*byte0

            events += 1

class RetinaView(object):
    def __init__(self, retina):
        self.retina = retina
        thread.start_new_thread(self.update_loop, ())

    def update_loop(self):
        import pylab
        import numpy
        pylab.ion()
        self.fig = pylab.figure()
        vmin, vmax = -1, 1
        self.img = pylab.imshow(self.retina.image, vmin=vmin, vmax=vmax,
                               cmap='gray', interpolation='none')
        self.line_y = pylab.vlines(self.retina.p_y, 0, 128)
        self.line_x = pylab.vlines(self.retina.p_x, 0, 128)
        while True:
            #self.fig.clear()
            #pylab.imshow(self.retina.image, vmin=vmin, vmax=vmax,
            #                   cmap='gray', interpolation='none')
            self.img.set_data(self.retina.image)
            #self.scatter[0].set_data([self.retina.p_x], [self.retina.p_y])
            #if len(self.retina.deltas) > 0:
            #    pylab.hist(self.retina.deltas, 20, range=(0.0, 0.1))
            #pylab.ylim(0, 100)
            #pylab.vlines(self.retina.p_y, 0, 128)
            #pylab.hlines(self.retina.p_x, 0, 128)

            y = self.retina.p_y
            self.line_y.set_segments(numpy.array([[(y,0), (y,128)]]))
            x = self.retina.p_x
            self.line_x.set_segments(numpy.array([[(0,x), (128,x)]]))

            self.retina.clear_image()
            pylab.draw()
            time.sleep(0.04)


if __name__ == '__main__':
    retina = Retina()
    view = RetinaView(retina)
    while True:
        time.sleep(1)
