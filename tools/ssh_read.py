"""Interactive SSH transport. Passwords are only held in memory, never logged."""
import getpass, re, shlex, time
import pexpect

def run(host, command, user='fairino', password=None, timeout=45, check=True):
    if not re.fullmatch(r'[A-Za-z0-9_.:-]+',host) or not re.fullmatch(r'[A-Za-z0-9_-]+',user):
        raise ValueError('Invalid SSH host/user')
    child=pexpect.spawn('ssh',['-T','-o','StrictHostKeyChecking=accept-new','-o','ConnectTimeout=8',
        '-o','ServerAliveInterval=5',user+'@'+host,'bash -c '+shlex.quote(command)],
        encoding='utf-8',timeout=timeout,maxread=65536)
    parts=[]; asked=False; deadline=time.monotonic()+timeout
    try:
        while True:
            remaining=max(.1,deadline-time.monotonic())
            event=child.expect([r'(?i)password:',pexpect.EOF,pexpect.TIMEOUT],timeout=remaining)
            parts.append(child.before)
            if event==1:
                child.close()
                if check and child.exitstatus not in (None,0):raise RuntimeError('SSH failed: '+''.join(parts)[-1500:])
                return ''.join(parts).replace('\r\n','\n')
            if event==2:raise TimeoutError('SSH timed out')
            if asked:raise RuntimeError('SSH authentication rejected')
            if password is None:password=getpass.getpass(f'SSH 密码 ({user}@{host}): ')
            child.sendline(password);asked=True
    finally:
        child.close(force=True)
