import serial
import time
import thread
import numpy as np

class Retina(object):
    def __init__(self, port='/dev/ttyUSB0'):
        self.serial = serial.Serial(port, baudrate=8000000, rtscts=True)
        time.sleep(0.5)
        self.serial.write("E+\nZ+\n")
        time.sleep(0.5)
        self.serial.write("E+\nZ+\n")
        self.image = np.zeros((128, 128), dtype=float)
        self.last_on = np.zeros((128, 128), dtype=float) + time.time()
        self.last_off = np.zeros((128, 128), dtype=float) + time.time()
        self.deltas = []
        thread.start_new_thread(self.update_loop, ())

    def clear_image(self):
        self.image *= 0.5
        del self.deltas[:]

    def update_loop(self):
        events = 0
        while True:
            byte0 = ord(self.serial.read())
            if byte0 & 0x80 == 0:
                continue
            byte0 = byte0 & 0x7F   # strip the top byte
            byte1 = ord(self.serial.read())
            sign = byte1 >= 0x7F
            byte1 = byte1 & 0x7F   # strip the top byte
            self.image[byte1, byte0] += 1 if sign else -1

            now = time.time()
            if sign:
                delta = now - self.last_off[byte1, byte0]
                self.last_on[byte1, byte0] = now
            else:
                delta = now - self.last_on[byte1, byte0]
                self.last_off[byte1, byte0] = now
            self.deltas.append(delta)

            events += 1

class RetinaView(object):
    def __init__(self, retina):
        self.retina = retina
        thread.start_new_thread(self.update_loop, ())

    def update_loop(self):
        import pylab
        pylab.ion()
        self.fig = pylab.figure()
        vmin, vmax = -1, 1
        #self.img = pylab.imshow(self.retina.image, vmin=vmin, vmax=vmax,
        #                       cmap='gray', interpolation='none')
        while True:
            #self.img.set_data(self.retina.image)
            self.fig.clear()
            if len(self.retina.deltas) > 0:
                pylab.hist(self.retina.deltas, 20, range=(0.0, 0.01))
            self.retina.clear_image()
            pylab.draw()
            time.sleep(0.02)


if __name__ == '__main__':
    retina = Retina()
    view = RetinaView(retina)
    while True:
        time.sleep(1)
