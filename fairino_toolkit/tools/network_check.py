"""Local, read-only routes and TCP reachability. Does not change NetworkManager."""
import json,subprocess,socket

def main():
    for command in (['ip','-br','-4','address'],['ip','route'],['ip','route','get','192.168.58.1'],['ip','route','get','192.168.58.3']):
        print('$ '+' '.join(command),flush=True)
        subprocess.run(command,check=False)
    for host,port in [('192.168.58.1',22),('192.168.58.3',22),('192.168.58.3',3000)]:
        try:
            with socket.create_connection((host,port),timeout=2):print(host,port,'reachable')
        except OSError as e:print(host,port,str(e))
if __name__=='__main__':main()
