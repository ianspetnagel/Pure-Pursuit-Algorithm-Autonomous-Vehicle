# Pure-Pursuit-Algorithm-Autonomous-Vehicle

**Introduction**
The pure pursuit path tracking algorithm is a widely used method in autonomous vehicle navigation. Its use has grown exponentially in recent years with greater effort to produce a fully autonomous vehicle. It has been effective in land-based navigation challenges. It uses a simple geometric technique with lookahead distance to update its position, understand its location in a local environment, and map itself to its target point. 

**Discussion**
Implementing the pure pursuit algorithm involved understanding its geometric basis and how it interacts with vehicle dynamics. For our project, this was a differential robot whose objective was to use line-based navigation and image recognition to maintain its position in the center of a lane and reach a target point. The algorithm calculated a path for the vehicle to follow, adjusting its trajectory based on the vehicle's current position and a predetermined lookahead distance. This lookahead distance was a critical parameter that significantly influenced the algorithm's performance. Rather than using the suggested definition for Pure Pursuit implementation, we introduced the function into the directional_control_wheels_speed function in our code. This can be seen below in Figure 1.


