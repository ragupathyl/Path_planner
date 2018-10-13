
#ifndef RANDOM_PLANNER_H
#define RANDOM_PLANNER_H

//Header files for boost and standard C++ libraries
#include <./boost/numeric/ublas/matrix.hpp>
#include <./boost/numeric/ublas/io.hpp>
#include <./boost/numeric/ublas/functional.hpp>
#include <iostream>
//#include <math.h>


//Structure to represent (x,y) locations
struct xy {
        int x;
        int y;
};

class Random_planner {
private:

//Private variables to store Random planner info
int max_step_number;   //Max_number of steps
boost::numeric::ublas::vector<xy> *path;  //pointer to path vector
bool verbose;   //Verbose statements during algorithm runtime

//Temporary variables used in search()
int surround_cells;   // Count number of surrounding cells rejected
xy current_pose;
int rand_num;
int steps;
bool exitloop;   //flag to check if move was made in loop
int loop_count;   // count number of loop runs
int repeat_threshold;   //Threshold number of cells to keep in memory



//Boolean function to check if expected move hits a wall or an obstacle
bool check_wall(boost::numeric::ublas::matrix<int> map,int x,int y);

//Function to check if the next cell is visited in the last repeat_threshold moves
bool check_repeat(int steps,int x,int y );

public:

//Constructor for Random_planner
Random_planner(int n,bool verbose_flag=false) : max_step_number(n),verbose(verbose_flag){
        path=new boost::numeric::ublas::vector<xy> (n+1); //Dynamically allocating path based on max_step_number
        steps=0;
        repeat_threshold=floor(sqrt(n));
        exitloop=false;
        loop_count=0;
        surround_cells=0;
}
//Destructor to delete dynamically allocated vector path
~Random_planner(){
        delete path;
}

//Search() function to implement Random planner
boost::numeric::ublas::vector<xy> search(boost::numeric::ublas::matrix<char> world_state, xy robot_pose, xy goal_pose);

//Function to print world_state with path
void printpath(boost::numeric::ublas::matrix<char> world_state);

//Function to print world_state
void printworld(boost::numeric::ublas::matrix<char> world_state,xy robot_pose, xy goal_pose);

};
#endif
