#!/usr/bin/env python3
"""Repair three API/variable typos in MWelle77 ROS-TCP-Endpoint main-ros2.

Run on the *source tree* before colcon build. Fail if upstream changes
unexpectedly rather than silently producing a broken container.
"""
from pathlib import Path
import ast
import sys

if len(sys.argv) != 2:
    raise SystemExit('usage: fix_tcp_sender.py /path/to/tcp_sender.py')

path = Path(sys.argv[1])
source = path.read_text(encoding='utf-8')
changes = [
    ('    def start_(self, conn, halt_event):',
     '    def start_sender(self, conn, halt_event):'),
    ('        _thread = threading.Thread(',
     '        sender_thread = threading.Thread('),
    ('            target=self._loop, args=(conn, self.sender_id, halt_event)',
     '            target=self.sender_loop, args=(conn, self.sender_id, halt_event)'),
]

for old, new in changes:
    if old in source:
        if source.count(old) != 1:
            raise RuntimeError(f'Unexpected match count for: {old!r}')
        source = source.replace(old, new, 1)
    elif new in source:
        print(f'Already fixed: {new.strip()}')
    else:
        raise RuntimeError(f'Upstream changed; cannot safely apply fix: {old.strip()}')

ast.parse(source)
cls = next((n for n in ast.parse(source).body
            if isinstance(n, ast.ClassDef) and n.name == 'UnityTcpSender'), None)
if cls is None:
    raise RuntimeError('UnityTcpSender class not found')
if not any(isinstance(m, ast.FunctionDef) and m.name == 'start_sender' for m in cls.body):
    raise RuntimeError('start_sender still missing')

path.write_text(source, encoding='utf-8')
print(f'Patched sender successfully: {path}')
