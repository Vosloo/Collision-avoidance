# Collision-avoidance for robotic manipulator with 3 DOF

Simulation of collision avoidance of manipulator with **3 DOF** in an environment with **static** obstacles.<br>
Avoidance is currently implemented only for end-effector of the manipulator.<br>
The entire simulation is written in **Python** using modules such as: 
* [robotics-toolbox-python](https://github.com/petercorke/robotics-toolbox-python) - toolbox providing useful functionalities for robotic-based simulations
* [autograd](https://github.com/HIPS/autograd) - set of mathematical functionalities such as: differentiation, backpropagation, etc.
* [swift](https://github.com/jhavl/swift) - browser-based simulator for robotic manipulators
* **numpy**
* **matplotlib**

Task of collision avoidance is mainly an **optimization problem** in which the **potential function**, controlling how the end-effector is moving, need to be **minimized**. Publications for further read:
* G. Pająk, I. Pająk, “Collision-free sub-optimal trajectory planning of the redundant manipulators”, Int. J. of Applied Mechanics and Engineering, 2010, vol.15, No.3, pp. 799-810, Institute of Computer Science and Production Management, University of Zielona Góra
* O. Khatib, “Real-time obstacle avoidance for manipulators and mobile robots”, Artificial Intelligence Laboratory Stanford University, Stanford, CA 94305
* K. Glass, Member, IEEE, R. Colbaugh, Member IEEE, D. Lim, and H. Seraji, Senior Member, IEEE, “Real-time collision avoidance for redundant manipulators” IEEE Transactions on robotics and automation, vol. 11, no. 3, June 1995
