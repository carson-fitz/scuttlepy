#!/usr/bin/python3

import numpy as np
from scuttlepy import SCUTTLE
from scuttlepy.constants import *

scuttle = SCUTTLE()

try:
    while True:
        print(scuttle.getGlobalPosition())
        scuttle.move(1)
        scuttle.turnPID(90)
        print(np.degrees(scuttle.getHeading()))
        scuttle.move(1)
        scuttle.turnPID(90)
        print(np.degrees(scuttle.getHeading()))
        scuttle.move(1)
        scuttle.turnPID(90)
        print(np.degrees(scuttle.getHeading()))
        scuttle.move(1)
        scuttle.turnPID(90)
        print(np.degrees(scuttle.getHeading()))
        print(scuttle.getGlobalPosition())

except KeyboardInterrupt:

    pass

finally:

    scuttle.stop()
