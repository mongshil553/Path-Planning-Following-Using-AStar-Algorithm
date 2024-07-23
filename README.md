# Path-Planning/Following-Using-AStar-Algorithm
Path Planning Using AStar Algorithm

This project's objective is to design an algorithm for a 2D car plan a path to its goal and to track it.
The ROS Server is given by the company Xytron.

<hr>
<h3>Path Planning</h3>
<h4>1. Transformation Matrix</h4>
The map coordinate system differs from that of the car. To make calculation simpler and with less error, the points in the car's coordinate system is transformed into the that of the map, and vice versa.

<h4>2. Next Node Candidate Selection</h4>
For a smooth turn, next nodes can only be on the 30 degrees left and right on both front and back. Say next nodes are selected 10px away from the car. We sample 4 nodes for both front and back, then these nodes are transformed into the map coordinate system. These nodes becomes the candidate nodes.

<h4>3. Heuristic Criteria</h4> <br>
The shortest path is not the optimum path since there can be irrational turns or the end position(rotation) might not meet the requirement.

<hr>
<h3>Path Following</h3>
<h4>1. Pure Pursuit</h4>
Drive angle is determined with pure pursuit algorithm.
<h4>2. P Control</h4>
Reference and error can be calculated by yaw with respect to map coordinate system. The yaw valuable is a global variable therefore no localization is needed. By using P Control of yaw error, the car can be driven to follow the path.
