**PSO-Simulation**
This code simulated PSO (Particle Swarm optimization) algorithm with five agents for path-finding.\
There are two classes: *(1) Init-map.py and (2)*.\
*Init-map.py* will create a grid map whose sizes are 10x10 cells. On the map, there are several obstacles (each obstacles is corresponding a cell) and the agent used proximity sensor to detect them. Initially, the obstacles are unknown. As agent detects a obstacles, it will transmit the obstacles' position to others agents to avoid.\
In each running, PSO algorithm would generate a different path to destination. The program runs on Python 3.4.
