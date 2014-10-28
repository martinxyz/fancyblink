#!/usr/bin/env python
import numpy as np

N = 2**12
t = np.arange(N).astype('f') / N
v = np.sin(t*np.pi)
v = (v*(2**32-1)).round()

f = open('halfsin_lut.c', 'w')
f.write('''
//generated file
const uint32_t halfsin_lut[%d] = {

''' % N)
for value in v:
    f.write('  %d,\n' % int(value))

f.write('};\n')

#from pylab import *
#plot(v)
#show()
