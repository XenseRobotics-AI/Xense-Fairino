#!/usr/bin/env python3
"""Bounded serial diagnostics; Modbus function 03 only, restore all settings."""
import os,termios,tty,select,time,struct,json,signal,fcntl,errno,copy,argparse
from pathlib import Path

parser=argparse.ArgumentParser()
parser.add_argument('--device',required=True)
parser.add_argument('--mode',choices=['passive','bms','candidate'],default='passive')
args=parser.parse_args()
assert args.device in ['/dev/ttyS0','/dev/ttyS3','/dev/ttyS4','/dev/ttyS6','/dev/ttyS7']
# Only the port explicitly specified by the installed BMS driver is actively queried.
assert args.mode!='bms' or args.device=='/dev/ttyS7'
assert args.mode!='candidate' or args.device in ['/dev/ttyS0','/dev/ttyS3','/dev/ttyS6']
owners=[]
for p in Path('/proc').iterdir():
 if not p.name.isdecimal() or int(p.name)==os.getpid():continue
 try:
  for f in (p/'fd').iterdir():
   try:
    if os.readlink(f)==args.device:owners.append(int(p.name));break
   except OSError:pass
 except OSError:pass
if owners:raise RuntimeError('Port in use: '+str(owners))

def crc(data):
 c=65535
 for b in data:
  c^=b
  for _ in range(8):c=(c>>1)^0xa001 if c&1 else c>>1
 return struct.pack('<H',c)
def request(address,count):
 p=struct.pack('>BBHH',address,3,0,count);return p+crc(p)
def interrupted(*_):raise InterruptedError('interrupted')
signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
fd=os.open(args.device,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
original=termios.tcgetattr(fd)
report={'device':args.device,'mode':args.mode,'trials':[],'rs485_flags':None}
try:
 try:
  flags=bytearray(32);fcntl.ioctl(fd,0x542e,flags,True);report['rs485_flags']=list(struct.unpack('8I',flags))
 except OSError as e:report['rs485_ioctl_error']=str(e)
 tty.setraw(fd,termios.TCSANOW)
 configs=[(115200,'N'),(9600,'N')] if args.mode=='passive' else [(115200,'N'),(9600,'N'),(19200,'N'),(38400,'N'),(57600,'N'),(9600,'E'),(19200,'E'),(115200,'E')]
 if args.mode=='candidate':configs=[(115200,'N'),(9600,'N'),(19200,'N')]
 for baud,parity in configs:
  settings=termios.tcgetattr(fd)
  settings[2]=(settings[2]&~(termios.PARENB|termios.PARODD|termios.CSTOPB|termios.CSIZE|termios.CRTSCTS))|termios.CS8|termios.CLOCAL|termios.CREAD
  if parity=='E':settings[2]|=termios.PARENB
  settings[4]=settings[5]=getattr(termios,'B'+str(baud));settings[6][termios.VMIN]=0;settings[6][termios.VTIME]=0
  trial={'baud':baud,'parity':parity,'sent':[]};report['trials'].append(trial)
  try:termios.tcsetattr(fd,termios.TCSANOW,settings)
  except termios.error as e:trial['error']=str(e);continue
  received=bytearray()
  def collect(seconds):
   until=time.monotonic()+seconds
   while time.monotonic()<until:
    ready,_,_=select.select([fd],[],[],min(.05,max(0,until-time.monotonic())))
    if ready:
     try:
      chunk=os.read(fd,4096)
      if chunk:received.extend(chunk)
      else:time.sleep(.005)
     except BlockingIOError:pass
    if len(received)>65536:break
  collect(1.2 if args.mode=='passive' else .2)
  trial['passive_bytes']=len(received)
  if args.mode in ['bms','candidate']:
   queries=[(1,45),(1,1),(2,1)] if args.mode=='bms' else [(1,45),(1,1)]
   for address,count in queries:
    q=request(address,count);assert q[1]==3
    os.write(fd,q);termios.tcdrain(fd);trial['sent'].append(q.hex());collect(.45)
  trial['received_bytes']=len(received);trial['hex_prefix']=received[:1000].hex();valid=[]
  for i in range(max(0,len(received)-4)):
   if received[i] not in [1,2] or received[i+1] not in [3,0x83]:continue
   size=5 if received[i+1]==0x83 else 5+received[i+2]
   frame=bytes(received[i:i+size])
   if len(frame)==size and crc(frame[:-2])==frame[-2:]:valid.append(frame.hex())
  trial['valid_modbus_frames']=valid
  if valid:break
finally:
 termios.tcsetattr(fd,termios.TCSANOW,original)
 report['settings_restored']=termios.tcgetattr(fd)==original
 os.close(fd)
print(json.dumps(report))
