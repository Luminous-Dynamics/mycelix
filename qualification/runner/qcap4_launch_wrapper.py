#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
from pathlib import Path

FD_ENV = "QCAP4_LAUNCH_FD"


def _write_all(fd, data):
    view = memoryview(data)
    while view:
        written = os.write(fd, view)
        if written <= 0:
            raise OSError("short launch handshake write")
        view = view[written:]


def _launch_fd():
    raw = os.environ.get(FD_ENV)
    try:
        fd = int(raw) if raw is not None else -1
    except ValueError:
        fd = -1
    if fd < 3:
        print("qcap4_launch_wrapper=launch handshake fd invalid", file=sys.stderr)
        raise SystemExit(20)
    return fd


def _signal(fd, line):
    try:
        _write_all(fd, line.encode("utf-8"))
    except OSError:
        pass


def _fail(fd, kind, message):
    _signal(fd, f"{kind}:{message}\n")
    print("qcap4_launch_wrapper=" + message, file=sys.stderr)
    raise SystemExit(20)


def _control(cgroup, name, fd):
    path = Path(cgroup) / name
    if path.is_symlink() or not path.is_file():
        _fail(fd, "ERR", "containment primitive missing " + name)
    return path


def _join_before_exec(cgroup, fd):
    procs = _control(cgroup, "cgroup.procs", fd)
    control_fd = None
    try:
        control_fd = os.open(procs, os.O_WRONLY | os.O_CLOEXEC)
        data = (str(os.getpid()) + "\n").encode("ascii")
        if os.write(control_fd, data) != len(data):
            _fail(fd, "ERR", "short cgroup.procs write")
    except OSError:
        _fail(fd, "ERR", "cannot join containment cgroup")
    finally:
        if control_fd is not None:
            try:
                os.close(control_fd)
            except OSError:
                pass
    try:
        members = {int(value) for value in procs.read_text().split() if value.isdigit()}
    except (OSError, UnicodeError):
        _fail(fd, "ERR", "cannot verify containment membership")
    if os.getpid() not in members:
        _fail(fd, "ERR", "containment membership not established")


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    fd = _launch_fd()
    if len(args) < 2:
        _fail(fd, "ERR", "usage: qcap4_launch_wrapper.py CGROUP EXEC [ARGS...]")
    cgroup = Path(args[0])
    executable = args[1]
    if not cgroup.is_absolute():
        _fail(fd, "ERR", "cgroup path must be absolute")
    if not os.path.isabs(executable):
        _fail(fd, "EXECERR", "gate executable must be absolute")

    _join_before_exec(cgroup, fd)
    _signal(fd, "MEMBER\n")
    try:
        os.close(fd)
    except OSError:
        pass
    environment = dict(os.environ)
    environment.pop(FD_ENV, None)
    try:
        os.execve(executable, args[1:], environment)
    except OSError:
        print("qcap4_launch_wrapper=gate exec failed", file=sys.stderr)
        raise SystemExit(21)


if __name__ == "__main__":
    main()
