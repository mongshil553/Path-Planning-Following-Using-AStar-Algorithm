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

