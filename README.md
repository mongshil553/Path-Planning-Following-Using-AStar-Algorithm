# Path-Planning/Following-Using-AStar-Algorithm
Path Planning Using AStar Algorithm

This project's objective is to design an algorithm for a 2D car plan a path to its goal and to track it.
The ROS Server is given by the company Xytron.

<hr>
<h3>Path Planning</h3>
<h4>1. Result</h4>
<kbd>
  <img src = "https://github.com/user-attachments/assets/4cb9a4a7-2ab1-48f7-a6f4-69a64c34f8ba" width="65%" height="65%"> &nbsp;
  <img src = "https://github.com/user-attachments/assets/407b1cf2-0e9c-4c2f-a501-f221bae28672" width="32.7%" height="32.7%">
</kbd>

<h4>2. Transformation Matrix</h4>
The map coordinate system differs from that of the car. To make calculation simpler and with less error, the points in the car's coordinate system is transformed into the that of the map, and vice versa.

<h4>3. Next Node Candidate Selection</h4>
For a smooth turn, next nodes can only be on the 30 degrees left and right on both front and back. Say next nodes are selected 10px away from the car. We sample 4 nodes for both front and back, then these nodes are transformed into the map coordinate system. These nodes becomes the candidate nodes.

<h4>4. Heuristic Criteria</h4>
The shortest path is not the optimum path since there can be irrational turns or the end yaw might not meet the requirement. There are 3 main criteria; <br><br>
<dl>
  <dd>
    i) Distance<br>
    Distance costs follow euclidian method.<br><br>
    ii) Error between current yaw and target end yaw.<br>
    The car trys to match the target end yaw. This means that car trys to make a path that can go straight to its target point and math the target end yaw.<br><br>
    iii) Intersection with the straight line to the end position and virtual barrier around goal. <br>
    The virtual barriers are created around the goal where only the entrance is open. The car trys to avoid going straight to the goal when there is a collision with the virtual barrier.> <br><br-->

    
  </dd>
</dl>
<!-- 조건 설명 필요함. i) 조건은 차가 무작전 목표 지점과 가까워지도록 하지만, 목표 각도, 자동차 하드웨어 한계를 벗어날 수 있음.  ii) 목표 각도로 차를 제어하려고 함. 이때 각도만 맞을 뿐, 직진을 했을 때 목표 지점과 offset이 발생할 수 있음. iii) ii)에서 발생하는 offset을 상쇄시켜줄 수 있음.-->

<hr>
<h3>Path Following</h3>
<h4>1. Result</h4>
<kbd>
  <img src = "https://github.com/user-attachments/assets/c27589ea-73f2-4a0d-b222-e3081e17e3bf" width="35%" height="35%"> &nbsp;
  <img src = "https://github.com/user-attachments/assets/62736a2e-3913-449e-a841-5a8e928bb5b5" width="36.2%" height="36.2%"> &nbsp;
  <img src = "https://github.com/user-attachments/assets/9c6d211a-c775-45a6-987c-0bdb5f72d51a" width="24.5%" height="24.5%"> &nbsp;
</kbd>


<h4>2. Angle Calculation</h4>
<!--Drive angle is determined with pure pursuit algorithm.-->
<h4>3. P Control</h4>
Reference and error can be calculated by yaw with respect to map coordinate system. The yaw valuable is a global variable therefore no localization is needed. By using P Control of yaw error, the car can be driven to follow the path.
