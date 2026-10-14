#!/usr/bin/env python3
"""Temporarily test supported RS485 direction modes on the documented BMS port."""
from pathlib import Path
import os,termios,tty,fcntl,struct,signal,time,select,json
DEVICE='/dev/ttyS7'
for p in Path('/proc').iterdir():
 if not p.name.isdecimal() or int(p.name)==os.getpid():continue
 try:
  for f in (p/'fd').iterdir():
   try:
    if os.readlink(f)==DEVICE:raise RuntimeError('ttyS7 already in use by '+p.name)
   except OSError:pass
 except OSError:pass

def interrupted(*_):raise InterruptedError('interrupted')
signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
fd=os.open(DEVICE,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
original=termios.tcgetattr(fd);rs=bytearray(32);fcntl.ioctl(fd,0x542e,rs,True);saved=bytes(rs)
r={'device':DEVICE,'original_flags':list(struct.unpack('8I',saved)),'trials':[]}
direction_changed=False
try:
 tty.setraw(fd,termios.TCSANOW)
 for flags in [3,5]:
  raw=bytearray(struct.pack('8I',flags,0,0,0,0,0,0,0))
  try:
   fcntl.ioctl(fd,0x542f,raw,True)
   direction_changed=True
  except OSError as e:r['trials'].append({'requested_flags':flags,'error':str(e)});continue
  check=bytearray(32);fcntl.ioctl(fd,0x542e,check,True)
  for baud in [115200,9600]:
   st=termios.tcgetattr(fd);st[2]=(st[2]&~(termios.PARENB|termios.PARODD|termios.CSTOPB|termios.CSIZE|termios.CRTSCTS))|termios.CS8|termios.CLOCAL|termios.CREAD
   st[4]=st[5]=getattr(termios,'B'+str(baud));st[6][termios.VMIN]=0;st[6][termios.VTIME]=0;termios.tcsetattr(fd,termios.TCSANOW,st)
   received=bytearray()
   for _ in range(2):
    q=bytes.fromhex('01030000002d85d7');assert q[1]==3;os.write(fd,q);termios.tcdrain(fd)
    until=time.monotonic()+.7
    while time.monotonic()<until:
     ready,_,_=select.select([fd],[],[],.05)
     if ready:
      try:
       chunk=os.read(fd,4096)
       if chunk:received.extend(chunk)
       else:time.sleep(.005)
      except BlockingIOError:pass
   r['trials'].append({'requested_flags':flags,'effective_flags':list(struct.unpack('8I',check)),'baud':baud,'received_bytes':len(received),'hex':received[:500].hex()})
finally:
 try:
  if direction_changed:fcntl.ioctl(fd,0x542f,bytearray(saved),True)
 finally:
  try:
   termios.tcsetattr(fd,termios.TCSANOW,original)
   check=bytearray(32);fcntl.ioctl(fd,0x542e,check,True)
   r['direction_restored']=bytes(check)==saved;r['termios_restored']=termios.tcgetattr(fd)==original
  finally:os.close(fd)
print(json.dumps(r))
