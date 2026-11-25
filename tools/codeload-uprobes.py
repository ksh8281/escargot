"""Track the unmodified CodeLoad executable in a private tracefs instance."""
from pathlib import Path
import os
import platform
import shutil
import struct
import subprocess


class Probes:
    def __init__(self, binary, cpu):
        self.root = Path('/sys/kernel/tracing')
        self.group = f'codeload_{os.getpid()}'
        self.instance = self.root / 'instances' / self.group
        self.added = []
        self.created = False
        self.binary = binary
        self.cpu = cpu

    @staticmethod
    def write(path, value, append=False):
        cmd = ['sudo', '-n', 'tee'] + (['-a'] if append else []) + [str(path)]
        subprocess.run(cmd, input=value+'\n', text=True, stdout=subprocess.DEVNULL,
                       stderr=subprocess.PIPE, check=True, timeout=5)

    def enable(self, offsets=None, diagnostics=None):
        if not shutil.which('sudo'):
            raise ValueError('sudo unavailable')
        if offsets is None:
            sym = subprocess.check_output(['nm','-an', self.binary], text=True)
            entries = [row.split() for row in sym.splitlines() if len(row.split()) == 3]
            code = {name:int(addr,16) for addr, kind, name in entries if kind in ('t','T')}
            with self.binary.open('rb') as stream:
                header = stream.read(64)
                phoff = struct.unpack_from('<Q',header,32)[0]
                size, count = struct.unpack_from('<HH',header,54)
                stream.seek(phoff)
                phdrs = [struct.unpack_from('<IIQQQQQQ',stream.read(size)) for _ in range(count)]
            def offset(address):
                segment, = [p for p in phdrs if p[0]==1 and p[3]<=address<p[3]+p[5]]
                return segment[2]+address-segment[3]
            names = dict(eval=next(s for s in code if 'L11builtinEvalE' in s and '.' not in s),
                         mark='GC_stopped_mark',grow='GC_expand_hp_inner')
            offsets = {event:offset(code[symbol]) for event,symbol in names.items()}
        reg = '%x0' if platform.machine()=='aarch64' else '%di'
        subprocess.run(['sudo','-n','mkdir',self.instance], capture_output=True, check=True, timeout=5)
        self.created=True
        self.write(self.instance/'tracing_on','0')
        self.write(self.instance/'trace_clock','mono')
        self.write(self.instance/'tracing_cpumask',f'{1 << self.cpu:x}')
        self.write(self.instance/'per_cpu'/f'cpu{self.cpu}'/'buffer_size_kb', '4096')
        for event, position in offsets.items():
            spec = f'p:{self.group}/{event} {self.binary}:0x{position:x}'
            variants = [spec+' blocks='+reg+':u64',spec] if event=='grow' else [spec]
            for definition in variants:
                try:
                    self.write(self.root/'uprobe_events',definition,True)
                    self.added.append(event)
                    break
                except subprocess.CalledProcessError as exc:
                    if diagnostics:
                        with diagnostics.open('a') as f:
                            print(event, repr(exc.stderr),definition,file=f)
            else:
                raise ValueError(f'kernel declined probe {event} at offset {position:x}')
        self.write(self.instance/'events'/self.group/'enable','1')

    def start(self):
        self.write(self.instance/'trace','')
        self.write(self.instance/'tracing_on','1')

    def stop(self, dest):
        self.write(self.instance/'tracing_on','0')
        with dest.open('wb') as f:
            subprocess.run(['sudo','-n','cat',self.instance/'trace'], stdout=f, check=True)

    def close(self):
        if self.created:
            subprocess.run(['sudo','-n','rmdir',self.instance],capture_output=True,check=True)
        for event in reversed(self.added):
            self.write(self.root/'uprobe_events',f'-:{self.group}/{event}',True)
