"""Bounded Linux pipe exchange shared by both simulation adapters."""

import math
import os
import select
import time


def exchange(process, fields, timeout):
    process.stdin.write(' '.join(str(v) for v in fields) + '\n')
    process.stdin.flush()
    deadline = time.monotonic() + timeout
    data = b''
    while b'\n' not in data:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([process.stdout], [], [], remaining)[0]:
            raise RuntimeError('C++ supervisor response timeout' + (' after a partial line' if data else ''))
        chunk = os.read(process.stdout.fileno(), 4096)
        if not chunk:
            raise RuntimeError('C++ supervisor closed its response pipe')
        data += chunk
        if len(data) > 1024:
            raise RuntimeError('C++ supervisor response exceeds 1024 bytes')
    if data.count(b'\n') != 1 or not data.endswith(b'\n'):
        raise RuntimeError('C++ supervisor emitted extra response data')
    try:
        reply = data.decode('ascii').split()
        if len(reply) != 6 or int(reply[0]) != fields[0]:
            raise ValueError('bad fields or sequence')
        mode, reason = reply[1:3]
        velocity = [float(v) for v in reply[3:]]
        if mode not in {'INIT', 'ACTIVE', 'HOLD', 'LAND', 'COMPLETE'}:
            raise ValueError('unknown mode')
        if not all(math.isfinite(v) for v in velocity) or math.hypot(*velocity) > 2.00001:
            raise ValueError('invalid velocity')
    except (ValueError, UnicodeError) as exc:
        raise RuntimeError('Malformed C++ supervisor response') from exc
    return mode, reason, velocity
