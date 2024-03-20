# Pure-Pursuit-Algorithm-Autonomous-Vehicle

**Introduction**


The pure pursuit path tracking algorithm is a widely used method in autonomous vehicle navigation. Its use has grown exponentially in recent years with greater effort to produce a fully autonomous vehicle. It has been effective in land-based navigation challenges. It uses a simple geometric technique with lookahead distance to update its position, understand its location in a local environment, and map itself to its target point. 

**Discussion**


Implementing the pure pursuit algorithm involved understanding its geometric basis and how it interacts with vehicle dynamics. For our project, this was a differential robot whose objective was to use line-based navigation and image recognition to maintain its position in the center of a lane and reach a target point. The algorithm calculated a path for the vehicle to follow, adjusting its trajectory based on the vehicle's current position and a predetermined lookahead distance. This lookahead distance was a critical parameter that significantly influenced the algorithm's performance. Rather than using the suggested definition for Pure Pursuit implementation, we introduced the function into the directional_control_wheels_speed function in our code. This can be seen below in Figure 1.

<img width="566" alt="1111" src="https://github.com/ianspetnagel/ianspet/assets/62821052/ebb5189f-a916-43f3-8c43-cc18b47278fe">

*Figure 1 - directional_control_wheels_speed Function*


The project used the Webots simulation environment where we were able to program the vehicle's controller and understand its simulation parameters. 
We are then calling the direction_control_wheels_speed( ) function in the main( ) to calculate the right and left wheels speed.

<img width="695" alt="22134" src="https://github.com/ianspetnagel/ianspet/assets/62821052/5e4debee-53d7-4964-8ad8-a0f125a962d1">


As we start the simulation the robot initially moves straight then uses the Lookahead distance we gave to change its course to turn and follow the left line. After a while, it will continue to turn but some calculations are causing the problem and the robot stops turning and goes straight until it crashes. 


**Conclusion**

In the context of Webots, where real-world testing may be limited, the simulation environment becomes a crucial testing ground for algorithms like pure pursuit. This virtual setting allows developers to fine-tune parameters and assess the algorithm's performance in diverse scenarios, paving the way for more robust and adaptable robotic systems.
While specific resources for implementing the pure pursuit algorithm in Webots were scarce, the general principles of the algorithm provided a foundation for its application in the WeBots simulation environment. The pure pursuit approach is important in path planning and helps understand your position and its association with your target point. 

As the field of robotics continues to evolve, the intersection of algorithms like pure pursuit and simulation environments like Webots becomes increasingly pivotal. Exploring the ongoing developments, potential advancements, and community-driven solutions in this space can provide a holistic perspective for developers and researchers venturing into the realm of robotic path planning.

**Other Photos of Webots simulation world**

<img width="876" alt="3333" src="https://github.com/ianspetnagel/ianspet/assets/62821052/5ca2a2f7-9f53-436e-9eeb-320f3d91946b">

<img width="917" alt="444" src="https://github.com/ianspetnagel/ianspet/assets/62821052/6b9c2fd6-145d-4bd2-92e2-82feee220ad9">






