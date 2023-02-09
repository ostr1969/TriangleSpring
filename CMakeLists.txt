#cmake ./ -D CMAKE_PREFIX_PATH=~/NEWOPENSIM



SET(CMAKE_CXX_COMPILER /usr/bin/g++)
#SET(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_STANDARD 17)
cmake_minimum_required(VERSION 3.2)
project(metabolic)
#set(OpenSimMoco_DIR /home/barako/NEWOPENSIM/cmake)
#set(OpenSim_DIR /home/barako/NEWOPENSIM/opensim-core/cmake)
project(predrun)



#set(source_files  RegisterTypes_TriangleSpring.cpp mocorun.cpp )
set(source_files   mocorun.cpp )
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenSim REQUIRED)
include_directories( 
	 /home/barako/NEWOPENSIM/include/
			 )

add_executable(predrun ${source_files} )
#target_link_libraries(predrun ${OpenSim_LIBRARIES} /usr/lib64/libstdc++.so.6  ~/NEWOPENSIM/lib64/libosimCommon.so ~/NEWOPENSIM/lib64/libosimMoco.so /home/barako/NEWOPENSIM/ipopt/lib/libipopt.so ~/NEWOPENSIM/adol-c/lib64/libadolc.so ~/NEWOPENSIM/casadi/lib/libcasadi.so ~/NEWOPENSIM/lib64/libosimActuators.so ~/NEWOPENSIM/simbody/lib64/libSimTKcommon.so ~/NEWOPENSIM/lib64/libosimSimulation.so ~/NEWOPENSIM/lib64/libosimTools.so  ~/NEWOPENSIM/lib64/libosimAnalyses.so )
target_link_libraries(predrun ${OpenSim_LIBRARIES} /usr/lib64/libstdc++.so.6  ~/NEWOPENSIM/lib64/libosimCommon.so ~/NEWOPENSIM/lib64/libosimMoco.so ~/NEWOPENSIM/casadi/lib/libcasadi.so ~/NEWOPENSIM/ipopt/lib/libipopt.so.1 ~/NEWOPENSIM/adol-c/lib64/libadolc.so.2)


