import serial
import socket
import time
import thread
import numpy as np

class Retina(object):
    def __init__(self, port='/dev/ttyUSB0'):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(('ctndroid.uwaterloo.ca', 56043))
        self.socket.settimeout(0)
        time.sleep(0.5)
        self.socket.send('!E1\nE+\n')
        time.sleep(0.5)
        self.socket.send('!E1\nE+\n')
        self.socket.send('!PA=20000\n!PA0=%50\n')

        self.image = np.zeros((128, 128), dtype=float)
        self.last_on = np.zeros((128, 128), dtype=np.uint16)
        self.last_off = np.zeros((128, 128), dtype=np.uint16)
        self.time_check = []
        self.p_x = 64.0
        self.p_y = 64.0
        self.deltas = []
        self.event_count = 0
        self.good_events = 0
        thread.start_new_thread(self.update_loop, ())


    def clear_image(self):
        self.image *= 0.5
        del self.deltas[:]

    def update_loop(self):
        events = 0
        old_data = None
        while True:
            try:
                data = self.socket.recv(1024)
                if len(data) % 4 != 0:
                    continue
                data_all = np.fromstring(data, np.uint8)

                data_x = data_all[::4]
                errors = np.where(data_x < 0x80)[0]
                if len(errors) > 0:
                    print 'error', len(errors)
                    old_data = data[errors[0]*4+1:]
                    continue

                data_x = data_x & 0x7F
                data_y = data_all[1::4]
                index_on = (data_y & 0x80) > 0
                index_off = (data_y & 0x80) == 0
                sign = np.where(index_on, 1, -1)
                data_y &= 0x7F
                self.image[data_x, data_y] += sign

                time = data_all[2::4].astype(np.uint16)
                time = (time << 8) + data_all[3::4]

                #time_check = time[(data_x == 110) & (data_y == 42)]
                #if len(time_check) > 0:
                #    self.time_check.extend(time_check)

                delta = np.where(index_on,
                                 time - self.last_off[data_x, data_y],
                                 time - self.last_on[data_x, data_y])

                self.last_on[data_x[index_on], data_y[index_on]] = time[index_on]
                self.last_off[data_x[index_off], data_y[index_off]] = time[index_off]

                eta = 0.2
                t_exp = 10000
                sigma_t = 1000
                t_diff = delta.astype(np.float) - t_exp
                try:
                    w_t = np.exp(-(t_diff**2)/(2*sigma_t**2))
                    #print (w_t>0.5).sum(), len(w_t)
                    self.event_count += len(w_t)
                    self.good_events += (w_t>0.5).sum()
                    r_x = np.average(data_x, weights=w_t)
                    r_y = np.average(data_y, weights=w_t)

                    self.p_x = (1-eta)*self.p_x + (eta)*r_x
                    self.p_y = (1-eta)*self.p_y + (eta)*r_y
                except ZeroDivisionError:
                    pass

                #self.deltas.extend(delta)



            except socket.error:
                pass

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
            #    pylab.hist(self.retina.deltas, 30)
            #pylab.ylim(0, 100)
            #pylab.vlines(self.retina.p_y, 0, 128)
            #pylab.hlines(self.retina.p_x, 0, 128)

            y = self.retina.p_y
            self.line_y.set_segments(numpy.array([[(y,0), (y,128)]]))
            x = self.retina.p_x
            self.line_x.set_segments(numpy.array([[(0,x), (128,x)]]))

            if self.retina.event_count > 0:
                print float(self.retina.good_events)/self.retina.event_count
            self.retina.good_events = 0
            self.retina.event_count = 0

            self.retina.clear_image()
            pylab.draw()
            time.sleep(0.04)


if __name__ == '__main__':
    retina = Retina()
    view = RetinaView(retina)
    while True:
        time.sleep(1)
