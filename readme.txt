CS 498 IR Final Project

--- Authors ---
Yanran Ding, Mengchao Zhang, Haoran Tang

--- Objective ---
Develope a sampling-based motion planning framework for legged robot to traverse challenging terrain

--- method ---
There are two layers. 
The lower layer is a locally optimal mixed-integer kinodynamic motion planning algorithm that acts as a steering function given start point and goal point.
The upper layer is a sampling-based algorithm that grows and keeps track of a roadmap or tree structure.

--- folders ---

__planner__: code for higher level sampling-based motion planning algorithm

__steering_fcn__: code for locally optimal steering function

docs: related publications, feel free to add more documents

results: intermediate results for progress update and group discussion

presentation: for final presentation
