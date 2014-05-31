#!/usr/bin/env python3
"""vcp - the virtual clockport for FS-UAE"""

import sysv_ipc
import time
import struct


class VCP:
    """wraps the virtual clockport protocol"""

    key = 0x68000

    # events
    EV_READ = 1
    EV_WRITE = 2
    EV_RESET = 3
    EV_QUIT = 4

    def __init__(self, handler):
        self.sig = sysv_ipc.Semaphore(self.key)
        self.ack = sysv_ipc.Semaphore(self.key+1)
        self.mem = sysv_ipc.SharedMemory(self.key)
        self.handler = handler

    def main_loop(self):
        # say hello to waiting FS-UAE
        self.mem.write(b'\x01', offset=11)
        # main loop
        stay = True
        while stay:
            # wait for event
            self.sig.P()

            # read event from shm
            data = self.mem.read()
            ev = data[8]
            reg = data[9]
            val = data[10]
            cycles = struct.unpack_from("=Q", data, 0)[0]

            if ev == self.EV_READ:
                val = self.handler.cp_get(cycles, reg)
                b = bytes([val])
                self.mem.write(b, offset=10)
            elif ev == self.EV_WRITE:
                self.handler.cp_put(cycles, reg, val)
            elif ev == self.EV_RESET:
                self.handler.cp_reset()
            elif ev == self.EV_QUIT:
                stay = False

            # confirm event
            self.ack.V()
        # say good bye
        self.mem.write(b'\x00', offset=11)


class VCPHandler:
    def cp_get(self, cycles, reg):
        pass

    def cp_put(self, cycles, reg, val):
        pass

    def cp_reset(self):
        pass


class PrintHandler(VCPHandler):
    def cp_get(self, cycles, reg):
        val = reg * 2
        print("%lx: cp_get(%d)->%02x" % (cycles, reg, val))
        return val

    def cp_put(self, cycles, reg, val):
        print("%lx: cp_put(%d, %02x)" % (cycles, reg, val))

    def cp_reset(self):
        print("RESET")


if __name__ == '__main__':
    # a simple example
    handler = PrintHandler()
    vcp = VCP(handler)
    vcp.main_loop()
