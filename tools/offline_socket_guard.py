"""Linux subprocess socket filter for the offline verification tools.

Pass ``install_socket_filter`` as ``subprocess.Popen(preexec_fn=...)``.  The
filter is inherited across exec/fork and blocks creation of IPv4, IPv6 and
packet sockets, including loopback, while leaving Unix sockets available.
An installation error raises before exec, so the child is never run without
the requested filter.  This is not a general sandbox: already-open sockets
must not be inherited (keep subprocess ``close_fds=True``), and Unix sockets
can still communicate with local services.
"""
from __future__ import annotations

import ctypes
import errno


_SCMP_ACT_ALLOW = 0x7FFF0000
_SCMP_ACT_ERRNO = 0x00050000 | errno.EPERM
_SCMP_CMP_MASKED_EQ = 7
_PR_SET_NO_NEW_PRIVS = 38
_BLOCKED_DOMAINS = (2, 10, 17)  # Linux AF_INET, AF_INET6, AF_PACKET.


class _ArgumentComparison(ctypes.Structure):
    _fields_ = [
        ('arg', ctypes.c_uint),
        ('op', ctypes.c_int),
        ('datum_a', ctypes.c_uint64),
        ('datum_b', ctypes.c_uint64),
    ]


# Resolve libraries and function signatures in the parent, before fork.  A
# missing library prevents the caller from launching a supposedly guarded run.
_seccomp = ctypes.CDLL('libseccomp.so.2', use_errno=True)
_libc = ctypes.CDLL(None, use_errno=True)
_seccomp.seccomp_init.argtypes = [ctypes.c_uint32]
_seccomp.seccomp_init.restype = ctypes.c_void_p
_seccomp.seccomp_syscall_resolve_name.argtypes = [ctypes.c_char_p]
_seccomp.seccomp_syscall_resolve_name.restype = ctypes.c_int
_seccomp.seccomp_rule_add_array.argtypes = [
    ctypes.c_void_p, ctypes.c_uint32, ctypes.c_int, ctypes.c_uint,
    ctypes.POINTER(_ArgumentComparison),
]
_seccomp.seccomp_rule_add_array.restype = ctypes.c_int
_seccomp.seccomp_load.argtypes = [ctypes.c_void_p]
_seccomp.seccomp_load.restype = ctypes.c_int
_seccomp.seccomp_release.argtypes = [ctypes.c_void_p]
_seccomp.seccomp_release.restype = None
_libc.prctl.argtypes = [ctypes.c_int, ctypes.c_ulong, ctypes.c_ulong,
                        ctypes.c_ulong, ctypes.c_ulong]
_libc.prctl.restype = ctypes.c_int


def _require_success(code: int, operation: str) -> None:
    if code < 0:
        raise OSError(-code, 'Offline socket filter failed: ' + operation)


def install_socket_filter() -> None:
    """Install the irreversible filter in a disposable child, or raise.

    Call through ``preexec_fn`` only from a single-threaded test runner; Python
    preexec callbacks are unsuitable for a multi-threaded host application.
    Do not call in the workbench or a long-lived process that needs networking.
    """
    context = _seccomp.seccomp_init(_SCMP_ACT_ALLOW)
    if not context:
        raise OSError(errno.ENOMEM, 'Could not allocate offline socket filter')
    try:
        for name in (b'socket', b'socketpair'):
            syscall = _seccomp.seccomp_syscall_resolve_name(name)
            if syscall < 0:
                raise RuntimeError('Offline filter cannot resolve ' + name.decode('ascii'))
            for domain in _BLOCKED_DOMAINS:
                # The kernel interprets domain as int.  Mask its lower 32 bits
                # so a raw syscall with nonzero high bits cannot evade the rule.
                comparison = _ArgumentComparison(0, _SCMP_CMP_MASKED_EQ, 0xFFFFFFFF, domain)
                _require_success(_seccomp.seccomp_rule_add_array(
                    context, _SCMP_ACT_ERRNO, syscall, 1, ctypes.byref(comparison)),
                    'add socket domain rule')
        if _libc.prctl(_PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) != 0:
            raise OSError(ctypes.get_errno(), 'Could not set no_new_privs for offline filter')
        _require_success(_seccomp.seccomp_load(context), 'load seccomp rules')
    finally:
        _seccomp.seccomp_release(context)
