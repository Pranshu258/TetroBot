# TetroBot: Simulating a Tetrahedral Robot
Harish Krupo KPS, Pranshu Gupta


## 1 Project Goal and Description
The goal of the project is to build a robot with a tetrahedron shaped body. The tetrahedron is spherically symmetric and has 4 legs, with 2 caplets each,attached to each of its vertices (hips). The tetrahedron rolls along one of its edge based on the user input direction. The user inputs directions as a sequence which contains {L,R,O} characters for left, right and opposite movement respectively. The edge along which the Tet rotates is decided based on the current pivot/corner. This corner is the vertex which landed after the
previous rotation.

## 2 Initial Phases
The development of this project was done in four phases, the first one implementing the tetrahedral tumbling motion for the bot on a planar mesh made of equilateral triangles, then the actual animation of this motion
was implemented in the second phase and the bending legs of the bot were implemented in third. This is the fourth phase and final phase of the tetrobot project.

## 2.1 Phase I
The main purpose of this phase was to compute the Next Equilateral Triangle for a given Current Equilateral Triangle & command and display it in the interface. 

## 2.2 Phase II

In phase 2, the bot’s body and it’s rolling motion were implemented. The motion was executed across three methods - TriggerMotionStart, HalfRollBotandTriggerMotionComplete. The TriggerMotion-
Start method computes the target ET on the mesh as per the given command, the HalfRollBot method
implements a LERP on for the rolling angleθ, so that bot is animated, and the TriggerMotionComplete
method completes the motion by setting the current layout to the target layout, so that the bot is ready to
execute the next command. A sinusoidal like smooothing is done on the time parametertfrom slow takeoff and landing of the bot:


### 2.3 Phase III

The bot’s legs and its animation are implemented in phase III. Two of the four legs act as pivot when the
bot tumbles. The flying foot goes for the NET’s head while the opposite foot moves towards the top. The
position of the flying feet were computed as follows (the computation for opposite foot is similar):
The initial locationDof the flying foot and its final locationDnextare known, the distanceRbetween
these pointsDandDnextare computed, and then to achieve a motion along a circular
arc, the center of the circle with radiusR, that lies in the vertical plane and passes throughDandDnextis
found. This allows the program to animate the flying foot on this arc by rotating it by an angle of 60 owith
time.

For each pair of hip and feet, we find out the point which is equidistant from both the hip and feet and
lies in the vertical plane above the ground, this point is the knee.
Now that the dynamics of the bot are implemented, the next phase can begin, where the bot will have a
task to complete.

## 3 Phase IV Objective

The objective of this phase of the project is to implement navigation for the bot towards a user defined goal
in the presence of obstacles. These obstacles will also be defined by the user before the navigation starts.
The two main challenges of this phase are obstacle detection and path finding. The next section describes
more about the challenges and solution used to solve it.

## 4 Challenges

### 4.1 Obstacle Detection

An obstacle is a cylinder of a user defined radius and center on the terrain. A bot can not step into a NET
if it overlaps with an obstacle. There can be three kinds of overlaps - one in which the obstacle
lies completely inside the triangle, other in which the obstacle contains one or more vertices of the triangle,
and finally one in which it’s center lies outside the triangle but the obstacle itself overlaps with an edge.
(Note: the case where the whole triangle lies inside the obstacle is a special instance of the second case, in
which all the vertices lie inside the cylinder.)

At each step we iterate over the existing obstacles and find out if a move to the candidate NET is valid
or not. We check for all the three corresponding conditions:

1. Check if the center of the obstacle lies inside the triangle.
2. Check if the obstacle contains one of the vertices of the triangle by comparing the obstacle radius and
    the distance between the center and the vertices.
3. Check if the perpendicular dropped on an edge has length lower than the obstacle radius and the point
    of drop lies on the edge.

### 4.2 Path Finding

The robot behaves like a real world robot would, moving towards the goal while trying to avoid obstacles in
apartially observablelandscape. At any step, the robot can only see the three possible NETs and it is


aware of the path it has taken so far i.e. the triangles it has visited, along with the current distance from
the goal. It can detect if a candidate NET has an obstacle when it is deciding the next move.
The program uses the euclidean distance as a measure to choose the next step from candidate steps -
the one which brings the bot closer to the goal. However, it may choose to move away from the goal in
order to avoid an obstacle, or if the corresponding candidate NET has already been visited, in order to avoid
oscillating on the same path. Thus, it simulates a depth first graph traversal with a heuristic to choose the
next node to expand on. Instead of pre calculating the whole path, the program calculates the path in pieces
and backtracks when needed.
Cases like taking a detour, backtracking, being blocked have been
handled. There is one case in which the goal itself is blocked, which may leave the bot confused.

## 5 Implementation

The program uses a state machine to keep track of various states during execution. It primarily operates
in two modes: Creation mode and Animation mode. The creation mode provides a user interface to create
the obstacles on the plane. The animation mode lets the user pick the goal using a mouse click and the bot
moves towards the goal. The program starts with the creation mode to let the user define the obstacles.

### 5.1 Creation Mode

The creation mode is further divided into:

- Center Selection mode:Lets the user pick the center of the cylinder with a mouse click
- Radius change mode: The user can use keys defined below to change the radius of the cylinder
    around the previously chosen center.


#### 5.1.1 Key Bindings

The Creation mode uses the following controls (mouse clicks/keybindings) to generate obstacles and switch
between the various modes:
```
click: On a mouse click, an obstacle is created at the point of click and with initial radius=10. The program
then switches into radius change mode.
```
```
k: Pressing ’k’ increments the radius of the cylinder by 10 units.
```
```
j: Pressing ’j’ decrements the radius of the cylinder by 10 units.
```
```
h: On h key press, the cylinder is finalized and the mode resets back to center selection to pick the next
cylinder center.
```
```
D: Pressing ’D’ toggles between Creation mode and Animation Mode. Once in animation mode, simply clicking anywhere on the terrain will mark the next goal towards which the tetrobot has to move. 
```
### 5.2 Animation Mode

In the animation mode, the program initially checks if we have a target to reach. The target is selected using
a mouse click. In Animation mode, the mouse press creates a green pole to indicate the target. Once the
goal is known, the below algorithm is used to reach it.

#### 5.2.1 Obstacle Detection and Step Selection

The program starts withboolean animationset to false. When the animation variable is false, the
checkMovemethod of the TetroBot is called which in turn calls the following functions to decide if the
animation needs to be triggered:

- checkInsideTile(pt A, pt B, pt C, Obstacle o): This function check if the point defined by
    o.centeris within the triangle created by pts A, B and C.
- checkIfTileIsObstacle(pt A, pt B, pt C): This function detects if the tile defined by the pts A,
    B and C contain one or more obstacles which fall into one of the categories as defined in 8.
- checkIfVisited(pt c): This checks if we have already visited a tile with the centroid c.

The next sections describes how checkMove uses the above defined functions to decide the path.

#### 5.2.2 Goal Detection
The checkMove method uses checkInsideTile to see if we are already at the goal. If yes, then it returns false to stop the animation loop. If no, then it uses the path finding algorithm described below.

#### 5.2.3 Path Finding

The path finding algorithm starts by identifying the next triangles that the bot can move into. For each
of the triangles it calculates if the distance of the centroid of the triangle to the target’s center. It uses
checkIfTileIsObstacleandcheckIfVisitedmethods to check if the tile is an obstacle or has been already
visited, respectively. It picks the next tile which has the shortest distance between the centroid and the goal,
given that it is neither an obstacle nor has been previously visited.

## 6 Inspiration

The idea was to make the robot act in a realistic manner, as much as possible. Another way of implementing
the solution would be create the complete path from start to end in thecheckMovemethod and then
start the animation. This would feel unnatural as it would avoid the boundaries from the beginning. The
implemented solution calculates the next step after each roll so that if the robot realizes that it is stuck at a
tile, it backtracks to find a different path.

## 7 Conclusion

This was a fun project with several challenging aspects in each phase. Building the Tetrobot and implementing
its animation involved concepts of three dimensional geometry and a lot of learning.


