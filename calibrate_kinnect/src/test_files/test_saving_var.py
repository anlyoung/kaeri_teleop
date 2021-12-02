#!/usr/bin/env python

import numpy as np

trans = [0,1,2]
rot = [4,5,6,7]

outfile = 'test'
arr = [trans,rot]
np.save(outfile,arr)
print("Done")