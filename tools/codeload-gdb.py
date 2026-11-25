# Loaded only in a GDB session already stopped at the benchmark's first instruction.
import gdb
import json
import os
import re
import time

pid=gdb.selected_inferior().pid
exe=os.readlink(f"/proc/{pid}/exe")
with open(f"/proc/{pid}/maps") as stream:
    base=next(int(line.split("-")[0],16) for line in stream
              if line.rstrip().endswith(" "+exe) and int(line.split()[2],16)==0)
output=open(__OUTPUT__,"w",buffering=1)
count=0
class Event(gdb.Breakpoint):
    def __init__(self,name,offset):
        super().__init__("*"+hex(base+offset),internal=True)
        self.name=name
    def stop(self):
        global count
        if self.name=="eval":
            count+=1
            if count%100 and count<1600:
                return False
        with open(f"/proc/{pid}/status") as stream:
            hwm=int(re.search(r"^VmHWM:\s+(\d+)",stream.read(),re.M)[1])
        blocks=int(gdb.parse_and_eval("$x0")) if self.name=="grow" else 0
        print(json.dumps([self.name,count,hwm,blocks]),file=output)
        return False
for name,offset in [("eval",0x22bcb0),("grow",0x3a6720),("mark",0x3a1980)]:
    Event(name,offset)
