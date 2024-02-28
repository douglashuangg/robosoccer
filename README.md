# robosoccer
Robot vs Human Soccer

- Building a real life 1v1 soccer robot with one fully autonomous robot and another one controlled by you!


### Developing the control logic with simulations
- First implementation of autonomous navigation simulated on Webots using path planning with Markov decision processes and path following with pure pursuit (green robot has logic implemented):

<img src="https://github.com/douglashuangg/robosoccer/blob/main/PurePursuit.gif" alt="GIF" width="600">


### Developing the physical robot and playing field
- Robots only have driving functionality, built on an ESP32 microcontroller
- 1m x 50cm field
- Camera overlooking field for localization of ball and robot, discretization algorithm to turn the field into a grid
- Processing done on laptop, commands to wheels sent over Bluetooth
