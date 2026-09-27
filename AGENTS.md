Aim of this project is to rework the Microduck robot project to use Unitree Digital Servos J288 in place of the Dynamixel XL330 M288 servos.

A MuJoCo based reinforcement learning project can be found in ../microduck_rl this has STL based geometry, robot model definitions, actuator models. Code necessary to train a RL policy for the robot to walk, etc.

The Better Actuator Model (BAM) repository is available in ../bam . We can find information on the j288 servo we are using in this project, that we'll need to model using BAM on the Unitree website at the following links
https://support.unitree.com/home/en/J288-S288%20Servo/overview
https://support.unitree.com/home/en/J288-S288%20Servo/Control_Mode
https://support.unitree.com/home/en/J288-S288%20Servo/motor_protocol

The robot firmware can be found in the ../microduck folder. This has the code needed to interface with all the hardware.

The docs/datasheets contains a collection of files that are relevant to hardware we'll need to use in this reworked microduck project. This includes datasheets and tech docs for the unitree servos we'll be using.

If you make a commit, do not add yourself as a co-contributor.
