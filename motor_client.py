import socket
import sys


class Motor:
    def __init__(self, s):
        self.ud = 0  # idk
        self.uq = 0
        self.id = 0
        self.iq = 0
        self.wr = 0
        self.pos = 0
        self.s = s  # socket

    def obs(self):
        # todo: work with bytes no need to convert
        line = self.s.recv(256).decode()
        line = line.split(',')
        if len(line) > 6:
            self.ud = line[3]  # time is not needed
            self.uq = line[4]
            self.id = line[1]
            self.iq = line[2]
            self.wr = line[5]
            self.pos = line[6]

    def reset(self):
        self.s.sendall(b'X')

    def control(self, ud, uq):
        self.ud = ud
        self.uq = uq

    def send_ctrl(self):
        pkg = '{},{}'.format(self.ud, self.uq)
        self.s.sendall(bytes(pkg, 'utf-8'))

    def log(self):
        print('currents> id=>{} A | iq=>{} A\nvoltages> ud=> {} V | uq=>{} V\nmotor> w_r => {}rad/sec | theta => {} degree'.format(
            self.id, self.iq, self.ud, self.uq, self.wr, self.pos))


host = '127.0.0.1'
port = 9099

ud = 5
uq = 5

data = [x for x in range(10000)]

s = None

for res in socket.getaddrinfo(host, port, socket.AF_UNSPEC, socket.SOCK_STREAM):
    af, socktype, proto, canonname, sa = res

    try:
        s = socket.socket(af, socktype, proto)
    except OSError as msg:
        s = None
        continue
    try:
        s.connect(sa)
    except OSError as msg:
        s.close()
        s = None
        continue
    break

if s is None:
    print("could not open sokcet")
    sys.exit(1)
with s:
    motor = Motor(s)  # create motor with socket
    motor.control(100, 200)  # set the control signals  ud, uq
    motor.send_ctrl()  # send it via socket
    motor.obs()  # get the data
    motor.log()
    motor.reset()
    motor.obs()  # get the data
    motor.control(50, 50)  # set the control signals  ud, uq
    motor.send_ctrl()  # send it via socket
    motor.obs()  # get the data
