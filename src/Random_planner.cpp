
#ifndef RANDOM_PLANNER_CPP
#define RANDOM_PLANNER_CPP

//Header files for boost and standard C++ libraries
#include "../headers/Random_planner.h"


//Boolean function to check if expected move hits a wall or an obstacle
bool Random_planner::check_wall(boost::numeric::ublas::matrix<int> map,int x,int y){
        if(x<0 || y<0 || (x>map.size1()-1) || (y>map.size2()-1)) return true;
        else if(map(x,y)=='1') return true;
        else return false;
}

//Function to check if the next cell is visited in the last repeat_threshold moves
bool Random_planner::check_repeat(int steps,int x,int y ){
        for (int i=1; i<=repeat_threshold; i++) {
                if(steps-i >= 0) {
                        if((*path)(steps-i).x==x && (*path)(steps-i).y==y) return true;
                }
        }
        return false;
}

//Search() function to implement Random planner
boost::numeric::ublas::vector<xy> Random_planner::search(boost::numeric::ublas::matrix<char> world_state, xy robot_pose, xy goal_pose)
{
        using namespace boost::numeric::ublas;
        using std::cout;
        using std::endl;
        using std::cin;

        //Update current_pose to starting location
        current_pose=robot_pose;

        //Print world_state properties if verbose
        if(verbose) {
                cout<<"Processing World state with following paramenters:"<<endl;
                cout<<"Number of rows = "<<world_state.size1()<<endl;
                cout<<"Number of columns = "<<world_state.size2()<<endl<<endl;

                cout<<"Starting position : "<<robot_pose.x<<","<<robot_pose.y<<endl;
                cout<<"Goal position : "<<goal_pose.x<<","<<goal_pose.y<<endl<<endl;
                printworld(world_state,robot_pose,goal_pose);
                cout<<endl;
        }
        //Add Start location to path
        path->insert_element(0,current_pose);
//Random a number between 1-4 to determine next move
        rand_num=rand()%4+1;
        srand (time(NULL));

//Loop till max_step_number is reached or if loop_count runs over a certain limit
        while(steps<max_step_number && loop_count<5*max_step_number) {

                loop_count++;
                if(verbose) {
                        cout<<endl<<"Current position : "<<current_pose.x<<","<<current_pose.y<<endl;
                        cout<< "randomed number = "<<rand_num<<endl;
                        cout<<"Surround_cells count = "<<surround_cells<<endl;
                }
                switch (rand_num) {
/* For each case,
   > Check if the move is valid(no wall or obstacle)
   > Check if the destination cell was visited in the last repeat_threshold steps
   > Move to the cell if both conditions are satisfied
   > If either condition fails, check the next cell in order
   > Keep a surround_cells count for each rejected move. When surround_cells>=4, make any valid move, ignoring repeat cells
 */

                case 1: //Checking the cell below for a possible move

                        if(verbose) cout<<"Checking the cell ("<<current_pose.x+1<<","<<current_pose.y<<") for a possible move"<<endl;
                        if(!check_wall(world_state,current_pose.x+1,current_pose.y)) {
                                if(!check_repeat(steps,current_pose.x+1,current_pose.y) || surround_cells>=4) {
                                        current_pose.x++;
                                        steps++;
                                        if(verbose) cout<<"Step "<<steps<<" is complete"<<endl<<"Moved to ("<<current_pose.x<<","<<current_pose.y<<")"<<endl;
                                        path->insert_element(steps,current_pose);
                                        break;
                                }
                                else surround_cells++;
                        }
                        else surround_cells++;

                        exitloop = true;
                        if(verbose) cout<<"Move could not be made: Checking next cell"<<endl;
                        rand_num++;
                        break;

                case 2://Checking the cell above for a possible move

                        if(verbose) cout<<"Checking the cell ("<<current_pose.x-1<<","<<current_pose.y<<") for a possible move"<<endl;
                        if(!check_wall(world_state,current_pose.x-1,current_pose.y)) {
                                if(!check_repeat(steps,current_pose.x-1,current_pose.y) || surround_cells>=4) {
                                        current_pose.x--;
                                        steps++;
                                        if(verbose) cout<<"Step "<<steps<<" is complete"<<endl<<"Moved to ("<<current_pose.x<<","<<current_pose.y<<")"<<endl;
                                        path->insert_element(steps,current_pose);
                                        break;
                                }
                                else surround_cells++;
                        }
                        else surround_cells++;

                        exitloop = true;
                        rand_num++;
                        if(verbose) cout<<"Move could not be made: Checking next cell"<<endl;
                        break;

                case 3://Checking the cell to the right for a possible move

                        if(verbose) cout<<"Checking the cell ("<<current_pose.x<<","<<current_pose.y+1<<") for a possible move"<<endl;
                        if(!check_wall(world_state,current_pose.x,current_pose.y+1)) {
                                if(!check_repeat(steps,current_pose.x,current_pose.y+1) || surround_cells>=4) {
                                        current_pose.y++;
                                        steps++;
                                        if(verbose) cout<<"Step "<<steps<<" is complete"<<endl<<"Moved to ("<<current_pose.x<<","<<current_pose.y<<")"<<endl;
                                        path->insert_element(steps,current_pose);
                                        break;
                                }
                                else surround_cells++;
                        }
                        else surround_cells++;

                        exitloop = true;
                        rand_num++;
                        if(verbose) cout<<"Move could not be made: Checking next cell"<<endl;
                        break;

                case 4://Checking the cell to the left for a possible move

                        if(verbose) cout<<"Checking the cell ("<<current_pose.x<<","<<current_pose.y-1<<") for a possible move"<<endl;
                        if(!check_wall(world_state,current_pose.x,current_pose.y-1)) {
                                if(!check_repeat(steps,current_pose.x,current_pose.y-1) || surround_cells>=4) {
                                        current_pose.y--;
                                        steps++;
                                        if(verbose) cout<<"Step "<<steps<<" is complete"<<endl<<"Moved to ("<<current_pose.x<<","<<current_pose.y<<")"<<endl;
                                        path->insert_element(steps,current_pose);
                                        break;
                                }
                                else surround_cells++;
                        }
                        else surround_cells++;

                        exitloop = true;
                        rand_num=1;
                        if(verbose) cout<<"Move could not be made: Checking next cell"<<endl;
                        break;
                }

//Use rand_num++ instead of new random number if move was rejected
                if(exitloop) {
                        exitloop=false;
                        continue;
                }

//Check if goal is reached
                if(current_pose.x==goal_pose.x && current_pose.y==goal_pose.y) {
                        path->resize(steps+1);
                        if(verbose) {
                                cout<<endl<<"Reached goal in "<<steps<<" steps"<<endl;
                                printpath(world_state);
                        }
                        return (*path);
                }

//Random new number and reset surround cells;
                rand_num=rand()%4+1;
                surround_cells=0;
        }
//If while loop ends before goal is reached, return empty path
        path->resize(0);
        if(verbose) cout<<endl<<"Goal could not be reached in "<<max_step_number<<" random steps"<<endl;
        return (*path);
}

//Function to print world_state with path
void Random_planner::printpath(boost::numeric::ublas::matrix<char> world_state){
        if(path->empty()) {
                std::cout<<std::endl<<"Path vector is empty"<<std::endl;
                return;
        }
        else{
                for(int i=1; i<path->size()-1; ++i) world_state((*path)(i).x,(*path)(i).y)='*';


                world_state((*path)(0).x,(*path)(0).y)='S';
                world_state((*path)((path->size()-1)).x,(*path)((path->size()-1)).y)='G';
        }
        for(int i=0; i<world_state.size1(); ++i) {
                for(int j=0; j<world_state.size2(); ++j) {
                        std::cout<<world_state(i,j)<<' ';

                }
                std::cout<<std::endl;
        }
}

//Function to print world_state
void Random_planner::printworld(boost::numeric::ublas::matrix<char> world_state,xy robot_pose, xy goal_pose){
        std::cout<<std::endl;

        world_state(robot_pose.x,robot_pose.y)='S';
        world_state(goal_pose.x,goal_pose.y)='G';
        for(int i=0; i<world_state.size1(); ++i) {
                for(int j=0; j<world_state.size2(); ++j) {
                        std::cout<<world_state(i,j)<<' ';

                }
                std::cout<<std::endl;
        }
}

#endif
