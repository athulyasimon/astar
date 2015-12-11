#A* Implementation

A* is a search algorithm commonly used for path finding around obstacles. The implementation of this algorithm was an assignment for our ME449-Robotic Manipulation class.

<img src="https://raw.githubusercontent.com/athulyasimon/astar/master/Astar.png" width=400px align="center" "/>

The planner takes in the following inputs and returns a graph diplaying the available path (if there is one)
* robot radius
* radii and coordinates of obstacles
* coordinates of potential nodes
* coordinates of start and goal nodes

There are two test cases provided at the end of the code. The first one uses a list of fixed obstacles and the second one will generate a list of random obstacles. 
