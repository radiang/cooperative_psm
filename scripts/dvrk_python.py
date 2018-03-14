#!/usr/bin/env python 

import dvrk
# Create a Python proxy for PSM1, name must match ros namespace


p = dvrk.psm('PSM2')
p.home()

sleep(0.1)
#p.move_joint_one(0.2, 0)