# Inertial Navigation System (INS)
An inertial navigation system (INS) will be programmed to combine inertial and auxiliary data to output velocity, heading, and position, with the intention of being utilized in mobile robotics applications. This system would provide sufficient data to accurately estimate the state of the robot for various purposes. Various techniques will be employed to provide measurements that are resistant to discretization errors. 

The hardware design of the system consists of a main processor, sensors, and wireless communication. The INS software will consist of a setup step to initialize sensors, and a main loop that will run indefinitely, responsible for processing sensor data and transforming it into the appropriate measurements. Custom software for the robotics application will be developed as a separate module and will run in parallel with the inertial calculations. 

Software will be built with practices such as object-oriented design, inversion of control, and single responsibility principle. Test driven development and automated testing will be employed to ensure that the code is kept continuously compliant as development of the system continues. The project aims to create a system that will enhance the capabilities of mobile robotics systems with advanced real-time data. 

_Associated with the University of Akron honors research project._
