**PSO-Simulation**\
This code simulated PSO (Particle Swarm optimization) algorithm with five agents for path-finding.\
There are two classes: *(1) Init-map.py and (2) PSOwithObstacles.py*.\
*Init-map.py* will create a grid map whose sizes are 10x10 cells. On the map, there are several obstacles and each obstacles is corresponding a cell.\
*PSOwithObstacles.py* contains PSO algorithm and control the agent movement. The agents use proximity sensor to detect the obstacles. Initially, the obstacles are unknown. As agent detects a obstacles, it will transmit the obstacles' position to others agents to avoid.\
In each running, PSO algorithm would generate a different path to destination. The program runs on Python 3.4.\

Link to the simulation: \
https://www.youtube.com/watch?v=le2tfh9zR28
