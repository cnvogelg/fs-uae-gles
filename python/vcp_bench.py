#!/usr/bin/env python3
"""a simple vcp test to measure the timing between clockport accesses"""

import uae.vcp


class BenchHandler(uae.vcp.VCPHandler):
    def __init__(self, max=100):
        self.max = max
        self.cp_reset()

    def cp_get(self, cycles, reg):
        if self.last is not None:
            delta = cycles - self.last
            self.deltas.append(delta)
            self.count += 1
        self.last = cycles

        if self.count == self.max:
            self._dump()
            self.cp_reset()

        return 0

    def cp_put(self, cycles, reg, val):
        self.cp_get(cycles, reg)

    def cp_reset(self):
        self.deltas = []
        self.last = None
        self.count = 0

    def _dump(self):
        mi = self.deltas[0]
        ma = mi
        avg = 0
        for delta in self.deltas:
            if delta > ma:
                ma = delta
            if delta < mi:
                mi = delta
            avg += delta
        avg /= len(self.deltas)
        mi = self._map(mi)
        ma = self._map(ma)
        avg = self._map(avg)
        print("min=%d max=%d avg=%d" % (mi, ma, avg))

    def _map(self, v):
        # convert to Amiga cycles
        # CYCLE_UNIT = 512
        v /= 512
        return v

if __name__ == '__main__':
    handler = BenchHandler()
    vcp = uae.vcp.VCP(handler)
    vcp.main_loop()
