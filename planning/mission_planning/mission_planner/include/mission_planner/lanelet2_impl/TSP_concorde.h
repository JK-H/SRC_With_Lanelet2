#ifndef MISSION_PLANNER_LANELET2_IMPL_TSP_CONCORDE_H
#define MISSION_PLANNER_LANELET2_IMPL_TSP_CONCORDE_H

#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <stdio.h>
#include <fstream>
#pragma once

//This class applies an object to solve a given TSP problem using the concorde TSP solver. This solver can be downloaded at:
//		http://www.math.uwaterloo.ca/tsp/concorde.html
//A short explanation on how to build the solver is given at:
//		http://www.math.uwaterloo.ca/tsp/concorde/DOC/README.html
//If you have build the solver navigate to the./TSP folder and type " ./concorde -h " to see how to use this solver. This class
//uses the concorde solver by using a systemcall.
//Make sure you have a "/common/files/TSP_order.txt" and a "/common/files/TSPlib_file.txt" file. These files are used to tell
//concorde the current problem and save the output of it.

using Mat = std::vector<std::vector<double>>;
class ConcordeTSPSolver {
   protected:
    std::string unique_file_identifier_;

	//Function to create neccessary TSPlib file to tell concorde what the problem is.
	void writeToFile(const Mat& pathlength_matrix, const std::string& tsp_lib_filename, const std::string& tsp_order_filename);

	//Function to read the saved TSP order.
	std::vector<int> readFromFile(const std::string& tsp_order_filename);

public:
	//Constructor
	ConcordeTSPSolver();

	//Functions to solve the TSP. It needs a distance matrix, that shows the pathlengths between two nodes of the problem.
	//This matrix has to be symmetrical or else the TSPlib must be changed. The int shows the index in the Matrix.

	//with given distance matrix
	std::vector<int> solveConcordeTSP(const Mat& path_length_Matrix);

};

#endif // !MISSION_PLANNER_LANELET2_IMPL_TSP_CONCORDE_H