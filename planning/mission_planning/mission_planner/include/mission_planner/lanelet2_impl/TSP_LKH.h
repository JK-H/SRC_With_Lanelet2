#ifndef MISSION_PLANNER_LANELET2_IMPL_TSP_LKH_H
#define MISSION_PLANNER_LANELET2_IMPL_TSP_LKH_H

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


using Mat = std::vector<std::vector<double>>;
class LKHTSPSolver {
   protected:
    std::string unique_file_identifier_;

	//Function to create neccessary TSPlib file to tell LKH what the problem is.
	void writeToFile(const Mat& pathlength_matrix, const std::string& tsp_lib_filename, const std::string& tsp_order_filename);

	//Function to read the saved TSP order.
	std::vector<int> readFromFile(const std::string& tsp_order_filename);

public:
	//Constructor
	LKHTSPSolver();

	//Functions to solve the TSP. It needs a distance matrix, that shows the pathlengths between two nodes of the problem.
	
    //with given distance matrix
	std::vector<int> solveLKHTSP(const Mat& path_length_Matrix);

};

#endif // !MISSION_PLANNER_LANELET2_IMPL_TSP_LKH_H