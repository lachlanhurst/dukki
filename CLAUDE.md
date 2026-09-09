Aim of this project is to rework the Microduck robot project to use Unitree Digital Servos J288 in place of the Dynamixel XL330 M288 servos.

A MuJoCo based reinforcement learning project can be found in ../microduck_rl this has STL based geometry, robot model definitions, actuator models. Code necessary to train a RL policy for the robot to walk, etc.

The robot firmware can be found in the ../microduck folder. This has the code needed to interface with all the hardware.

The docs/datasheets contains a collection of files that are relevant to hardware we'll need to use in this reworked microduck project. This includes datasheets and tech docs for the unitree servos we'll be using.
