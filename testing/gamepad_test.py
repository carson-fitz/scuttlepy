import time
import numpy as np
from gamepad import Gamepad
from scuttlepy import SCUTTLE

# Create gamepad object
gamepad = Gamepad()

# Create SCUTTLE Object
scuttle = SCUTTLE()

try:

    # Create infinite loop
    while True:

        # Get joystick x and y values and scale values to -1 to 1
        joystickX, joystickY = [((-2/255)*gamepad.axes['LEFT_X'])+1,
                                ((-2/255)*gamepad.axes['LEFT_Y'])+1
                                ]

        motion = [
                  # Mutiply joystick y axis by SCUTTLE max linear velocity to get linear velocity
                  joystickY*scuttle.maxVelocity,

                  # Mutiply joystick x axis by SCUTTLE max angular velocity to get angular velocity
                  joystickX*scuttle.maxAngularVelocity
                 ]

        scuttle.setMotion(motion)               # Set motion to SCUTTLE

        # Get the SCUTTLE's latest location
        x,y = scuttle.getGlobalPosition()

        # Print the location of the SCUTTLE
        # print('Global Position: {}, {}'.format(round(x, 3), round(y, 3)))

        # print(round(np.rad2deg(np.arctan(scuttle.getHeading()))))
        print(round(scuttle.getHeading(),3))

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:

    pass

finally:
    # Clean up.
    gamepad.close()
    scuttle.stop()
    print('Stopped.')