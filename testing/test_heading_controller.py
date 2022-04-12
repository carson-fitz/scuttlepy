#!/usr/bin/python3

import numpy as np
from scuttlepy import SCUTTLE
from scuttlepy.constants import *

scuttle = SCUTTLE()

try:

    while True:

        scuttle.setHeading(np.pi/2)
        #print(scuttle.getGlobalPosition())
        # print(scuttle.getHeading())

except KeyboardInterrupt:

    pass

finally:

    scuttle.stop()
