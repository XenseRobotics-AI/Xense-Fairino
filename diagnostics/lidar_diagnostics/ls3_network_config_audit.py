"""MAC-addressed network configuration matching SDKELI LS3Tool 1.1.1.

Discovery: frmEthernetConnect .ctor + udpFindThreadMethod.
Write: frmEthernetConnect.btIPConfigurateUdp_Click, 34-byte broadcast.
The only write is the user-authorized IP/subnet/gateway configuration.
"""
import json, socket, struct, select, time

QUERY = bytes.fromhex('fa5aa5aa040081008001')
PLANS = [
    ('02:50:c2:1e:00:45', '20260907-000046', '192.168.192.100'),
    ('02:50:c2:31:00:41', '20260907-000045', '192.168.192.101'),
]
report = {'steps': [], 'errors': [], 'source_ip': '192.168.192.7'}
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
tx.bind(('192.168.192.7', 0))
tx.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, b'eth0\0')
cap = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3))
cap.setblocking(False)

def receive(seconds):
    rows=[];seen=set();deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        ready,_,_=select.select([cap],[],[],min(.2,max(0,deadline-time.monotonic())))
        if not ready:continue
        d,a=cap.recvfrom(65535)
        if a[2]==4 or len(d)<42 or d[12:14]!=b'\x08\x00' or d[23]!=17:continue
        off=14+(d[14]&15)*4
        if len(d)<off+8:continue
        sp,dp,n,_=struct.unpack('!HHHH',d[off:off+8]);p=d[off+8:off+n]
        if sp!=30716 or p[:4]!=b'\xfa\x5a\xa5\xaa' or len(p)<10:continue
        key=(d[6:12],p)
        if key in seen:continue
        seen.add(key)
        row={'source_mac':d[6:12].hex(':'),'source_ip':socket.inet_ntoa(d[26:30]),
             'payload_hex':p.hex(),'command':p[8],'subcommand':p[9],
             'checksum_ok':(sum(p[8:])&65535)==int.from_bytes(p[6:8],'little'),
             'length_ok':int.from_bytes(p[4:6],'little')+6==len(p)}
        if p[8:10]==b'\x80\x01' and len(p)>=114:
            b=p[10:]
            row.update(model=b[:40].split(b'\0')[0].decode(errors='replace'),
                       serial=b[40:72].split(b'\0')[0].decode(errors='replace'),mac=b[72:78].hex(':'),
                       ip=socket.inet_ntoa(b[78:82]),mask=socket.inet_ntoa(b[82:86]),
                       gateway=socket.inet_ntoa(b[86:90]),data_port=int.from_bytes(b[90:92],'little'),
                       ntp_ip=socket.inet_ntoa(b[100:104]))
        rows.append(row)
    return rows

def discover():
    tx.sendto(QUERY,('255.255.255.255',30716))
    return [x for x in receive(2) if 'mac' in x and x['checksum_ok'] and x['length_ok']]

def make_config(old,new_ip,gateway):
    # byte 8:9 = command 0x80 / subcommand 0x02
    # byte 10:15 = target MAC; 16:19 IP; 20:23 mask; 24:27 gateway;
    # byte 28:29 = data port (LE); byte 30:33 = NTP server IP.
    data=b'\x80\x02'+bytes.fromhex(old['mac'].replace(':',''))
    data+=socket.inet_aton(new_ip)+socket.inet_aton(old['mask'])+socket.inet_aton(gateway)
    data+=struct.pack('<H',old['data_port'])+socket.inet_aton(old['ntp_ip'])
    packet=b'\xfa\x5a\xa5\xaa'+struct.pack('<HH',len(data)+2,sum(data)&65535)+data
    assert len(packet)==34
    return packet

try:
    before=discover();report['before']=before
    devices={x['mac']:x for x in before}
    for mac,serial,new_ip in PLANS:
        d=devices.get(mac)
        if not d or d['source_mac']!=mac or d['serial']!=serial or d['model']!='LS3-2027DE' or d['ip']!='192.168.0.10':
            raise RuntimeError('device identity or original address changed; refusing to write')
    # Recheck prospective addresses with read-only ARP immediately before writes.
    raw=socket.socket(socket.AF_PACKET,socket.SOCK_RAW);raw.bind(('eth0',0))
    own=bytes.fromhex(open('/sys/class/net/eth0/address').read().strip().replace(':',''))
    for _,_,ip in PLANS:
        for _ in range(3):
            p=struct.pack('!HHBBH',1,0x0800,6,4,1)+own+socket.inet_aton('192.168.192.7')+b'\0'*6+socket.inet_aton(ip)
            raw.send(b'\xff'*6+own+b'\x08\x06'+p)
            until=time.monotonic()+.25
            while time.monotonic()<until:
                ready,_,_=select.select([cap],[],[],.05)
                if not ready:continue
                packet,addr=cap.recvfrom(65535)
                if addr[2]!=4 and len(packet)>=42 and packet[12:14]==b'\x08\x06' and packet[28:32]==socket.inet_aton(ip):
                    raise RuntimeError('new address has an ARP responder: '+ip)
    raw.close()
    for mac,serial,new_ip in PLANS:
        old=devices[mac]
        packet=make_config(old,new_ip,'0.0.0.0')
        step={'mac':mac,'serial':serial,'old':old,'new_ip':new_ip,'new_gateway':'0.0.0.0',
              'write_packet_hex':packet.hex(),'rollback_packet_hex':make_config(old,old['ip'],old['gateway']).hex()}
        report['steps'].append(step)
        tx.sendto(packet,('255.255.255.255',30716))
        step['sent']=True
        step['acknowledgements']=receive(11)
        after=discover();step['discovery_after']=after
        match=[x for x in after if x['mac']==mac and x['serial']==serial and x['ip']==new_ip]
        if not match:
            after=discover();step['discovery_retry']=after
            match=[x for x in after if x['mac']==mac and x['serial']==serial and x['ip']==new_ip]
        if not match:raise RuntimeError('new IP not confirmed; stopped before changing any further device')
        step['verified']=True
        # Ensure the other sensor was not modified by the MAC-addressed command.
        expected={m:('192.168.192.100' if m==PLANS[0][0] else '192.168.0.10') for m,_,_ in PLANS}
        if mac==PLANS[0][0]:
            second=[x for x in after if x['mac']==PLANS[1][0] and x['ip']==expected[PLANS[1][0]]]
            if not second:raise RuntimeError('second device original IP not confirmed after first write')
    report['final']=discover()
except Exception as e:
    report['errors'].append(repr(e))
finally:
    tx.close();cap.close()
    print(json.dumps(report,ensure_ascii=False))
