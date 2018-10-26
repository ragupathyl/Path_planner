///////////////////////////////////////////
/* Path_planner libraries- Quick guide  */
//////////////////////////////////////////

Created by Lakshmi Naarayanan Ragupathy:

github link: https://github.com/ragupathyl/Path_planner


The file layout is organized as follows:

<current_directory>/boost    -  Contains the necessary boost/numeric/ublas directory with all its .hpp files
                   
<current_directory>/headers  -  Contains the Class definition files for classes Astar_planner and Random_planner

<current_directory>/src      -  Contains image files used in the documentaion.pdf
                   
<current_directory>/src      -  Contains the source files test_Astar_planner.cpp,Astar_planner.cpp,test_Random_planner.cpp and Random_planner.cpp

-------------------------------------------------------------------------------------------------
|The search() functions are defined in their respective source files in <current_directory>/src |
|For Analysis on the libraries, refer to section 6 of doumentation.pdf                          |
-------------------------------------------------------------------------------------------------


Testing:
--------
This project is fully self contained with boost files included.(Requires make and g++ to compile) 

1)To start with, run $make in the <current_directory> 

2)If compilation is successful, it creates two executable files in the <current_directory>
     test_Astar_planner.out and test_Random_planner.out

3)To test the Random_planner library run 

    >  $ ./test_Random_planner.out to generate a 10X10 map randomly with starting pose (1,1) and goal pose (8,8) and get user defined output 
    >  $ ./test_Random_planner.out argv[1] argv[2] argv[3] argv[4] argv[5] argv[6] argv[7] argv[8] to customize map size and other parameters
          argv[1] - row count, argv[2] - column count, (argv[3],argv[4]) starting pose, 
          (argv[5],argv[6]) goal pose, argv[7] max_step_number, argv[8]- bool Verbose  
          Additionally, the ratio of obstacles to open spaces can be set using int ratio;
   Or, edit ./src/test_Random_planner.cpp to set map manually 

3)To test the Astar_planner library run 

    >  $ ./test_Astar_planner.out to generate a 10X10 map randomly with starting pose (1,1) and goal pose (8,8) and get user defined output 
    >  $ ./test_Astar_planner.out argv[1] argv[2] argv[3] argv[4] argv[5] argv[6] argv[7] to customize map size and other parameters
          argv[1] - row count, argv[2] - column count, (argv[3],argv[4]) starting pose, 
          (argv[5],argv[6]) goal pose, argv[7] max_step_number
	  Additionally, the ratio of obstacles to open spaces can be set using int ratio;
   Or, edit ./src/test_Astar_planner.cpp to set map manually 
