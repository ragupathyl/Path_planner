
#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

   //Header files for boost and standard C++ libraries
 #include <./boost/numeric/ublas/matrix.hpp>
 #include <./boost/numeric/ublas/io.hpp>
 #include <./boost/numeric/ublas/functional.hpp>
 #include <iostream>
 #include <map>
 #include <queue>




//Structure to represent (x,y) locations
struct xy {
        int x;
        int y;
};

class Astar_planner {
private:

//Astar_planner state variable
bool verbose; //Verbose statements during algorithm runtime

//Map properties
int row;
int col;

//1D index of start and goal location
int start_idx;
int goal_idx;

//Structure to store processed cell properties
struct cell {
        int previous; //index of parent cell with shortest distance from start position
        int f;
        int h;
        int g;
};

// 1D boolean vector to mark cells that are added to closed list referenced by 1D index of cell
boost::numeric::ublas::vector<bool> *closedlist;

// Internal map to store cell properties with 1D index as key
std::map<int,cell> cellinfo;
std::map<int,cell>::iterator cell_it; //temp iterator for use in search()
std::map<int,cell>::iterator it; //temp iterator for use in tracepath()

//Priority queue to store pairs of <f(cell),cell_index> in openlist sorted in ascending order of F(cell)
std::priority_queue<std::pair<int,int>,std::vector<std::pair<int,int> >,std::greater <std::pair<int,int> > > openlist;

//Temp variables used in algorithm
int idx;
xy current_xy;
int f_current,g_current,h_current;
cell temp;
bool reached_goal;

//Vector of xy to store path from start to goal
boost::numeric::ublas::vector<xy> *path;

//Function to evaluate H(cell)
int Hvalue(xy a);

//Funtion to convert (x,y) to 1D index
int xy2idx(xy XY);

//Function to convert 1D index to (x,y)
xy idx2xy(int index);

//Function to check if the current move hits a wallor if it hits an obstacle
bool check_wall(boost::numeric::ublas::matrix<char> map,int idx1,int idx2);

//Function to trace path via cellinfo member 'previous'
boost::numeric::ublas::vector<xy> tracepath(std::map<int,cell> cellinfo,boost::numeric::ublas::matrix<char> world_state);

public:

//Constructor to set verbose option, default = false
Astar_planner(bool verbose_flag=false) : verbose(verbose_flag) {
}

//Destructor to delete dynamic vectors
~Astar_planner(){
        delete path;
        delete closedlist;
}

//Search function implementation
boost::numeric::ublas::vector<xy> search(boost::numeric::ublas::matrix<char> world_state, xy robot_pose, xy goal_pose);

//Function to print path from start to goal with arrows
void printpath(boost::numeric::ublas::matrix<char> world_state);

//Function to print World state
void printworld(boost::numeric::ublas::matrix<char> world_state,xy robot_pose, xy goal_pose);


};

#endif
