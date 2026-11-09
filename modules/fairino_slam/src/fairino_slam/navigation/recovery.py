"""Coordinate desktop session exit before one explicit remote recovery attempt.

This module does not touch GTK. The GUI reserves the local chassis, stops its
sessions, then grants a recovery lease when ready_to_acquire is set. No recovery
request is sent before that grant or while a previous navigation command runs.
"""
import threading
import time


def _recover(host, user, password):
    from fairino_slam.maintenance.recovery import run_recovery
    return run_recovery(host, user, password, execute=True)


class RecoverySession:
    def __init__(self, host, user, password, command_lock, local_ready,
                 runner=None, wait_timeout=45., clock=time.monotonic):
        if wait_timeout <= 0:
            raise ValueError('恢复等待时间必须大于零')
        self.host, self.user, self.password = host, user, password
        self.command_lock, self.local_ready = command_lock, local_ready
        self.runner, self.wait_timeout, self.clock = runner or _recover, wait_timeout, clock
        self.ready_to_acquire = threading.Event()
        self.authorized = threading.Event()
        self.done = threading.Event()
        self.rejected = None
        self.report, self.error = None, None
        self.message = '正在结束旧底盘会话，等待已有导航请求完成…'
        self.thread = threading.Thread(target=self.run, daemon=True)

    def authorize(self):
        self.authorized.set()

    def reject(self, reason):
        self.rejected = str(reason)
        self.authorized.set()

    def _remaining(self, deadline):
        remaining = deadline - self.clock()
        if remaining <= 0:
            raise RuntimeError('旧导航请求或底盘会话仍未退出；未接管底盘，请等待退出后重试恢复')
        return min(.05, remaining)

    def run(self):
        acquired = False
        try:
            deadline = self.clock() + self.wait_timeout
            while not acquired:
                acquired = self.command_lock.acquire(timeout=self._remaining(deadline))
            while not self.local_ready():
                self.done.wait(self._remaining(deadline))
            self.ready_to_acquire.set()
            while not self.authorized.wait(self._remaining(deadline)):
                pass
            self._remaining(deadline)
            if self.rejected:
                raise RuntimeError(self.rejected)
            if not self.local_ready():
                raise RuntimeError('底盘会话状态已变化，未发送恢复请求')
            self.message = '正在恢复底盘接入：停止遗留控制、清错并确认下使能…'
            # Exactly one call. An ambiguous SSH result is never retried here.
            report = self.runner(self.host, self.user, self.password)
            if not isinstance(report, dict) or not isinstance(report.get('ok'), bool):
                raise RuntimeError('未收到有效恢复结果，执行状态不明；请先复查，未自动重试')
            self.report = report
            if not report['ok']:
                self.error = report.get('failure') or report.get('blocked_reason') or '恢复检查未通过'
        except Exception as exc:
            self.error = str(exc) or type(exc).__name__
        finally:
            if acquired:
                self.command_lock.release()
            self.password = None
            self.done.set()
