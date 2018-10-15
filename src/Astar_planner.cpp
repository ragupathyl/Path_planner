
#ifndef PATH_PLANNER_ASTAR_CPP
#define PATH_PLANNER_ASTAR_CPP

//Header files for boost and standard C++ libraries
#include "../headers/Astar_planner.h"

//Function to evaluate H(cell)
int Astar_planner::Hvalue(xy a){
        current_xy=idx2xy(goal_idx);
        return (abs(a.x-current_xy.x)+abs(a.y-current_xy.y));
}

//Funtion to convert (x,y) to 1D index
int Astar_planner::xy2idx(xy XY){
        return (XY.x*col+XY.y);
}

//Function to convert 1D index to (x,y)
xy Astar_planner::idx2xy(int index){
        return (xy){floor(index/col),index%col};
}

//Function to check if the current move hits a wallor if it hits an obstacle
bool Astar_planner::check_wall(boost::numeric::ublas::matrix<char> map,int idx1,int idx2){
        xy temp=idx2xy(idx2);
        int x=temp.x;
        int y=temp.y;

        if(x<0 || y<0 || (x>map.size1()-1) || (y>map.size2()-1)) return true;
        else if((idx2==idx1-1) && (idx1%col==0)) return true;
        else if((idx2==idx1+1) && (idx2%col==0)) return true;
        else if(map(x,y)=='1') return true;
        else return false;
}

//Function to trace path via cellinfo member 'previous'
boost::numeric::ublas::vector<xy> Astar_planner::tracepath(std::map<int,cell> cellinfo,boost::numeric::ublas::matrix<char> world_state){

        it = cellinfo.find(goal_idx);
        int steps= it->second.g;

        path = new boost::numeric::ublas::vector<xy>(steps+1);

        for(int i=steps; i>=0; --i) {
                path->insert_element(i,idx2xy(it->first));
                it=cellinfo.find(it->second.previous);
        }
        if(verbose) printpath(world_state);
        return (*path);
}

//Search function implementation
boost::numeric::ublas::vector<xy> Astar_planner::search(boost::numeric::ublas::matrix<char> world_state, xy robot_pose, xy goal_pose)
{
        using namespace boost::numeric::ublas;
        using std::cout;
        using std::endl;
        using std::cin;
        using std::pair;
        using std::map;
        using std::priority_queue;
        using std::greater;
        using std::make_pair;
//Updating private state variables
        row=world_state.size1();
        col=world_state.size2();

        goal_idx=xy2idx(goal_pose);
        start_idx=xy2idx(robot_pose);


//Allocating closedlist 1D vector based on row*column size
        closedlist = new vector<bool> (row*col,false);

//Cout commands for verbose flag
        if(verbose) {
                cout<<"Processing World state with following paramenters:"<<endl;
                cout<<"Number of rows = "<<row<<endl;
                cout<<"Number of columns = "<<col<<endl<<endl;

                cout<<"Starting position : "<<robot_pose.x<<","<<robot_pose.y<<endl;
                cout<<"Goal position : "<<goal_pose.x<<","<<goal_pose.y<<endl<<endl;
                printworld(world_state,robot_pose,goal_pose);
                cout<<endl;
        }

//Adding properties of Starting cell into cellinfo
        h_current=Hvalue(idx2xy(start_idx));
        temp=(cell){start_idx,h_current,h_current,0};
        cellinfo.insert(pair<int,cell>(start_idx,temp));


//Pushing <0,start_dx> into open list; F=0 to ensure maximum priority;
        openlist.push(make_pair(0,start_idx));
//Setting flag variable
        reached_goal=false;

//Cout statement for verbose option
        if(verbose) cout<<"Entering while loop to explore map:"<<endl<<endl;

//Running while loop till openlist is emptied
        while(!openlist.empty()) {

//Popping the highest priority pair from openlist
                pair<int,int> current = openlist.top();
                openlist.pop();

                idx = current.second;
//Adding current cell to closedlist
                (*closedlist)(idx)=true;



                g_current = cellinfo.find(idx)->second.g;
//Cout statements for verbose option
                if(verbose) {
                        current_xy=idx2xy(idx);

                        cout<<"Exploring cell ("<<current_xy.x<<","<<current_xy.y<<")"<<endl;
                        cout<<"G value is = "<<g_current<<endl;
                        cout<<"H value is = "<<Hvalue(current_xy)<<endl;
                        cout<<"F value is = "<<current.first<<endl<<endl;
                }
//Checking the cell to the right, Proceed if it the move is valid
                if(!check_wall(world_state,idx,idx+1)) {
//If goal cell is found, update cellinfo and return tracepath()
                        if(idx+1==goal_idx) {
                                temp=(cell){idx,g_current+1,0,g_current+1};
                                cellinfo.insert(pair<int,cell>(idx+1,temp));
                                reached_goal=true;
                                if(verbose) cout<<"Goal reached in "<<g_current+1<<" steps"<<endl;
                                return tracepath(cellinfo,world_state);
                        }
//Else if cell isnt part of closedlist, calcualte F(cell)
                        else if((*closedlist)(idx+1)==false) {

                                h_current = Hvalue(idx2xy(idx+1));
                                f_current = g_current+1+h_current;
                                cell_it=cellinfo.find(idx+1);
//If cell isnt part of cellinfo, add to openlist and add cellinfo
                                if(cell_it==cellinfo.end()) {
                                        openlist.push(make_pair(f_current,idx+1));
                                        temp=(cell){idx,f_current,h_current,g_current+1};
                                        cellinfo.insert(pair<int,cell>(idx+1,temp));

                                }
//If cell is part of cellinfo, check if F(cell) can be updated with new value. Update cellinfo and openlist
                                else if(cell_it->second.f > f_current) {
                                        openlist.push(make_pair(f_current,idx+1));

                                        cell_it->second.previous=idx;
                                        cell_it->second.f=f_current;
                                        cell_it->second.g=g_current+1;
                                }

                        }
                }
//Checking the cell to the left, Proceed if it the move is valid
                if(!check_wall(world_state,idx,idx-1)) {
//If goal cell is found, update cellinfo and return tracepath()
                        if(idx-1==goal_idx) {
                                temp=(cell){idx,g_current+1,0,g_current+1};
                                cellinfo.insert(pair<int,cell>(idx-1,temp));
                                reached_goal=true;
                                if(verbose) cout<<"Goal reached in "<<g_current+1<<" steps"<<endl;
                                return tracepath(cellinfo,world_state);
                        }
//Else if cell isnt part of closedlist, calcualte F(cell)
                        else if((*closedlist)(idx-1)==false) {

                                h_current = Hvalue(idx2xy(idx-1));
                                f_current = g_current+1+h_current;
                                cell_it=cellinfo.find(idx-1);
//If cell isnt part of cellinfo, add to openlist and add cellinfo
                                if(cell_it==cellinfo.end()) {
                                        openlist.push(make_pair(f_current,idx-1));
                                        temp=(cell){idx,f_current,h_current,g_current+1};
                                        cellinfo.insert(pair<int,cell>(idx-1,temp));

                                }
//If cell is part of cellinfo, check if F(cell) can be updated with new value. Update cellinfo and openlist
                                else if(cell_it->second.f > f_current) {
                                        openlist.push(make_pair(f_current,idx-1));

                                        cell_it->second.previous=idx;
                                        cell_it->second.f=f_current;
                                        cell_it->second.g=g_current+1;
                                }

                        }
                }
//Checking the cell below, Proceed if it the move is valid
                if(!check_wall(world_state,idx,idx+col)) {
//If goal cell is found, update cellinfo and return tracepath()
                        if(idx+col==goal_idx) {
                                temp=(cell){idx,g_current+1,0,g_current+1};
                                cellinfo.insert(pair<int,cell>(idx+col,temp));
                                reached_goal=true;
                                if(verbose) cout<<"Goal reached in "<<g_current+1<<" steps"<<endl;
                                return tracepath(cellinfo,world_state);
                        }
//Else if cell isnt part of closedlist, calcualte F(cell)
                        else if((*closedlist)(idx+col)==false) {

                                h_current = Hvalue(idx2xy(idx+col));
                                f_current = g_current+1+h_current;
                                cell_it=cellinfo.find(idx+col);
//If cell isnt part of cellinfo, add to openlist and add cellinfo
                                if(cell_it==cellinfo.end()) {
                                        openlist.push(make_pair(f_current,idx+col));
                                        temp=(cell){idx,f_current,h_current,g_current+1};
                                        cellinfo.insert(pair<int,cell>(idx+col,temp));

                                }
//If cell is part of cellinfo, check if F(cell) can be updated with new value. Update cellinfo and openlist
                                else if(cell_it->second.f > f_current) {
                                        openlist.push(make_pair(f_current,idx+col));

                                        cell_it->second.previous=idx;
                                        cell_it->second.f=f_current;
                                        cell_it->second.g=g_current+1;
                                }

                        }
                }

//Checking the cell above, Proceed if it the move is valid
                if(!check_wall(world_state,idx,idx-col)) {
//If goal cell is found, update cellinfo and return tracepath()
                        if(idx-col==goal_idx) {
                                temp=(cell){idx,g_current+1,0,g_current+1};
                                cellinfo.insert(pair<int,cell>(idx-col,temp));
                                reached_goal=true;
                                if(verbose) cout<<"Goal reached in "<<g_current+1<<" steps"<<endl;
                                return tracepath(cellinfo,world_state);
                        }
//Else if cell isnt part of closedlist, calcualte F(cell)
                        else if((*closedlist)(idx-col)==false) {
//g_current += 1;
                                h_current = Hvalue(idx2xy(idx-col));
                                f_current = g_current+1+h_current;
                                cell_it=cellinfo.find(idx-col);
//If cell isnt part of cellinfo, add to openlist and add cellinfo
                                if(cell_it==cellinfo.end()) {
                                        openlist.push(make_pair(f_current,idx-col));
                                        temp=(cell){idx,f_current,h_current,g_current+1};
                                        cellinfo.insert(pair<int,cell>(idx-col,temp));

                                }
//If cell is part of cellinfo, check if F(cell) can be updated with new value. Update cellinfo and openlist
                                else if(cell_it->second.f > f_current) {
                                        openlist.push(make_pair(f_current,idx-col));

                                        cell_it->second.previous=idx;
                                        cell_it->second.f=f_current;
                                        cell_it->second.g=g_current+1;
                                }

                        }
                }

        }
//IF goal was not found, return empty path
        if(!reached_goal) {
                if(verbose) {
                        cout<<"Goal could not be reached "<<endl;
                        printworld(world_state,robot_pose,goal_pose);
                }
                path=new boost::numeric::ublas::vector<xy>();
                return (*path);
        }



}

//Function to print path from start to goal with arrows
void Astar_planner::printpath(boost::numeric::ublas::matrix<char> world_state){
        if(path->empty()) {
                std::cout<<std::endl<<"Path vector is empty"<<std::endl;
                return;
        }
        else{
                for(int i=1; i<path->size()-1; ++i) {
                        if((*path)(i+1).x-(*path)(i).x==1) world_state((*path)(i).x,(*path)(i).y)='v';
                        if((*path)(i+1).x-(*path)(i).x==-1) world_state((*path)(i).x,(*path)(i).y)='^';
                        if((*path)(i+1).y-(*path)(i).y==1) world_state((*path)(i).x,(*path)(i).y)='>';
                        if((*path)(i+1).y-(*path)(i).y==-1) world_state((*path)(i).x,(*path)(i).y)='<';
                }

                world_state((*path)(0).x,(*path)(0).y)='S';
                world_state((*path)((path->size()-1)).x,(*path)((path->size()-1)).y)='G';
        }
        for(int i=0; i<row; ++i) {
                for(int j=0; j<col; ++j) {
                        std::cout<<world_state(i,j)<<' ';

                }
                std::cout<<std::endl;
        }
}

//Function to print World state
void Astar_planner::printworld(boost::numeric::ublas::matrix<char> world_state,xy robot_pose, xy goal_pose){
        std::cout<<std::endl;

        world_state(robot_pose.x,robot_pose.y)='S';
        world_state(goal_pose.x,goal_pose.y)='G';
        for(int i=0; i<row; ++i) {
                for(int j=0; j<col; ++j) {
                        std::cout<<world_state(i,j)<<' ';

                }
                std::cout<<std::endl;
        }
}

#endif
