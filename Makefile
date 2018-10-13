#
# 'make depend' uses makedepend to automatically generate dependencies
#               (dependencies are added to end of Makefile)
# 'make'        build executable file 'mycc'
# 'make clean'  removes all .o and executable files
#

# define the C compiler to use
CC = g++

# define any compile-time flags
#CFLAGS =

# define any directories containing header files other than /usr/include
INCLUDES = -I./headers
#-I./src -I./headers -I./data -I./tools

# define library paths in addition to /usr/lib
LFLAGS= 


# define any libraries to link into executable:
#   if I want to link in libraries (libx.so or libx.a) I use the -llibname
#   option, something like (this will link in libmylib.so and libm.so:
LIBS = 


# define the executable file
V0 = test_Astar_planner
V1 = Astar_planner
V2 = test_Random_planner
V3 = Random_planner

#sourcepath
SRC = ./src/


#Building executables
all:    $(V0).out $(V2).out
	@echo  Compiled all files

$(V0).out : $(SRC)$(V0).cpp $(SRC)$(V1).cpp
	$(CC) $(INCLUDES) $(LFLAGS) -o $(V0).out $(SRC)$(V0).cpp $(SRC)$(V1).cpp $(LIBS)

$(V2).out : $(SRC)$(V2).cpp $(SRC)$(V3).cpp
	$(CC) $(INCLUDES) $(LFLAGS) -o $(V2).out $(SRC)$(V2).cpp $(SRC)$(V3).cpp $(LIBS)


#clean command to remove .out files and default .dat files
clean: 
	$(RM) *.out 




