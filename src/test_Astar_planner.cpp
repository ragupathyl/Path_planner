#include <./boost/numeric/ublas/matrix.hpp>
#include <./boost/numeric/ublas/io.hpp>
#include <./boost/numeric/ublas/assignment.hpp>
#include <iostream>
#include <string.h>
#include <time.h>
#include "../headers/Astar_planner.h"



int main(int argc, char* argv[]) {
        using boost::numeric::ublas::vector;
        using boost::numeric::ublas::matrix;
        using std::cin;
        using std::cout;
        using std::endl;

//Map parameters
        int row =20;
        int col=20;
//Start and end points
        xy start = {1,1};
        xy end={row-2,col-2};
//Set true if random map has to be generated
        bool random_map = true;
        int ratio=4; //Ratio of empty spaces to each obstacle rounded to nearest integer
        srand (time(NULL));
//Set verbose option for Planner Class
        bool verbose = false;


//Check for command line inputs;
        if(argc==8) {
                row=atoi(argv[1]);
                col=atoi(argv[2]);
                start.x=atoi(argv[3]);
                start.y=atoi(argv[4]);
                end.x=atoi(argv[5]);
                end.y=atoi(argv[6]);
                std::string flag=argv[7];
                if(flag.compare("true")==0) verbose=true;
                if(flag.compare("false")==0) verbose=false;
        }

//Declare map variable
        matrix<char> map(row,col);

//Generate random map
        if(random_map) {
                for(int i=0; i<row; i++)
                        for(int j=0; j<col; j++) {
                                if(rand()%ratio) map(i,j)='0';
                                else map(i,j)='1';
                        }
                map(start.x,start.y)='0';
                map(end.x,end.y)='0';
        }
//Or initialize map manually
        else{
                map <<='1','1','1','0','0','0','0','0','1','0','0','1','0','0','0','0','1','0','0','0',
                       '1','S','1','0','0','0','0','0','0','0','0','0','0','1','0','0','0','1','1','1',
                       '0','0','1','0','0','0','0','0','0','0','1','0','0','1','0','1','0','0','1','0',
                       '0','0','1','1','0','0','0','0','0','0','0','0','1','0','1','0','1','0','0','0',
                       '0','0','1','1','0','0','0','1','1','0','0','0','1','0','1','1','0','0','0','0',
                       '0','0','0','0','0','0','0','0','1','1','0','0','0','0','0','0','1','1','0','1',
                       '0','0','1','0','0','1','0','0','0','0','1','0','0','0','1','1','1','1','0','1',
                       '1','1','0','0','0','0','0','0','0','1','1','1','0','1','0','0','1','1','1','0',
                       '0','1','0','1','0','0','1','0','0','0','0','0','0','0','0','1','1','0','1','0',
                       '0','1','0','1','1','0','0','1','0','0','0','1','0','1','1','0','0','0','1','0',
                       '0','0','1','1','1','0','1','1','0','1','0','0','1','0','0','1','1','1','1','0',
                       '0','0','0','0','0','0','0','0','1','0','1','0','0','1','0','0','0','0','0','0',
                       '1','0','0','1','0','0','0','1','0','0','0','0','1','0','0','1','0','0','0','0',
                       '0','0','0','1','0','1','0','0','0','1','0','0','0','0','1','0','1','0','0','0',
                       '0','1','1','0','0','0','0','0','0','0','0','0','1','0','0','0','0','1','1','0',
                       '1','0','0','0','1','0','0','1','0','1','0','0','1','0','1','0','0','0','0','0',
                       '1','0','0','1','0','0','0','1','0','0','0','1','0','0','0','0','0','0','0','0',
                       '0','0','1','0','0','1','0','0','0','1','1','1','0','0','1','0','0','1','0','0',
                       '0','0','0','0','0','0','1','0','1','0','0','0','0','0','0','0','1','0','G','0',
                       '0','0','0','0','1','1','0','0','1','0','1','0','0','0','0','0','0','0','1','0';
        }


//Create an instance of Planner class
        Astar_planner obj(verbose);


//Call search() and obtain path
        vector<xy> path = obj.search(map,start,end);


        if(!verbose) {
                cout<<endl<<"World State:"<<endl;
                obj.printworld(map,start,end);
        }

        if(!verbose) {
                std::cout<<endl<<"Path coordinates in order:"<<endl;
                for(int i=0; i<path.size(); i++) cout<<path(i).x<<"  "<<path(i).y<<endl;


                if(!path.empty()) cout<<endl<<"Goal reached in "<<path.size()-1<<" steps"<<endl<<endl;
                obj.printpath(map); //Call printpath() to print map with optimal path marked
        }
        return 0;

}
