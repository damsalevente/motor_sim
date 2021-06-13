import socket
import sys


host = '127.0.0.1'
port = 9099

ud = 5
uq = 5

s = None

for res in socket.getaddrinfo(host,port,socket.AF_UNSPEC, socket.SOCK_STREAM):
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
    pkg = '{},{}'.format(ud,uq)
    s.sendall(bytes(pkg, 'utf-8'))
    data = s.recv(128)

print("received:", repr(data))
