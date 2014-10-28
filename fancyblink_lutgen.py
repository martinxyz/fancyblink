#!/usr/bin/env python
import numpy as np

t = np.linspace(0, 1, 1001)
v = -np.cos(t*2*np.pi)/2+0.5
v = v ** 2.2 # gamma correction
#v = v ** 1.7 # gamma correction
v = v*1000 + np.random.rand(len(v)) # dithering

f = open('fancyblink_lut.c', 'w')
# // static const int lut[10001] 
# //__attribute__ ((section (".rodata")))
f.write('''
//generated file
const int lut[1001] = {

''')
for value in v:
    f.write('  %d,\n' % int(value))

f.write('};\n')
