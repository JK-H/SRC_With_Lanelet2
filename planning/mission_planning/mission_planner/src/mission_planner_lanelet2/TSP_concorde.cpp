#include<mission_planner/lanelet2_impl/TSP_concorde.h>
#include <sys/time.h>
#include <assert.h>

//Default constructor
ConcordeTSPSolver::ConcordeTSPSolver(){}

//This function generates a file with the current TSP in TSPlib format. This is necessary because concorde needs this file
//as input to solve the TSP. See http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95/ for documentation.
void ConcordeTSPSolver::writeToFile(const Mat& pathlength_matrix, const std::string& tsp_lib_filename, const std::string& tsp_order_filename)
{
	const std::string path_for_saving_file = tsp_lib_filename;   //ros::package::getPath("libconcorde_tsp_solver") + "/common/files/TSPlib_file.txt";
	std::ofstream saving_file(path_for_saving_file.c_str());
	if (saving_file.is_open())
	{
		std::cout << "Starting to create the TSPlib file: " << path_for_saving_file << std::endl;
		//specify name of the Problem, Type (TSP = symmetrical TSP) and add a comment to the file. Name and Type are necessary, comment is for better understanding when you open the file.
		saving_file << "NAME: routing-with-lanelet2_" << tsp_lib_filename << std::endl
				<< "TYPE: TSP" << std::endl
				<< "COMMENT: This is the TSPlib file for using concorde."
				<< std::endl;
		saving_file << "DIMENSION: " << pathlength_matrix.size() << std::endl; //Shows the Dimension of the problem --> the number of nodes (Necessary).
		//Write the distance-matrix into the file as a full-matrix.
		saving_file << "EDGE_WEIGHT_TYPE: EXPLICIT" << std::endl;
		saving_file << "EDGE_WEIGHT_FORMAT: FULL_MATRIX" << std::endl;
		saving_file << "EDGE_WEIGHT_SECTION" << std::endl;

		for (int row = 0; row < pathlength_matrix.size(); row++)
		{
			for (int col = 0; col < pathlength_matrix[0].size(); col++)
			{
				saving_file << " "
						    << (int) pathlength_matrix[row][col];
			}
			saving_file << std::endl;
		}
		//shows the end of the file
		saving_file << "EOF";

		std::cout << "Created the TSPlib file." << std::endl;
		saving_file.close();
	}
	else
	{
		std::cout << "Saving file '" << path_for_saving_file << "' for concorde could not be opened." << std::endl;
	}

	// clear results file
	std::ofstream reading_file(tsp_order_filename.c_str()); //open file
	if (reading_file.is_open())
	{
		reading_file << "";
		reading_file.close();
	}
	else
	{
		std::cout << "Could not clear results file '" << tsp_order_filename << "'." << std::endl;
	}
}

//This function opens the file which saves the output from the concorde solver and reads the saved order. The names of the
//nodes in the graph are stored as positions in the distance matrix in this case. The first integer in the file is the number
//of nodes of this problem, so this one is not necessary.
std::vector<int> ConcordeTSPSolver::readFromFile(const std::string& tsp_order_filename)
{
	std::string path_for_order_file = tsp_order_filename; //ros::package::getPath("libconcorde_tsp_solver") + "/common/files/TSP_order.txt"; //get path to file
	std::ifstream reading_file(path_for_order_file.c_str()); //open file

	std::vector<int> order_vector; //vector that stores the calculated TSP order

	std::string line; //current line of the file

	int value; //current node in the line

	int line_counter = 0; //variable to make sure that the first line isn't stored

	if (reading_file.is_open())
	{
		//get new line in the file
		while (getline(reading_file, line))
		{
			std::istringstream iss(line);
			while (iss >> value)
			{
				if (line_counter > 0) //first line shows the number of nodes and is not relevant
					order_vector.push_back(value); //put the current node in the last position of the order
				line_counter++;
			}
		}
		reading_file.close();
	}
	else
	{
		std::cout << "TSP order file '" << path_for_order_file << "' could not be opened." << std::endl;
	}
	return order_vector;
}

//This function solves the given TSP using the systemcall to use the concorde TSP solver. This solver is applied from:
//		http://www.math.uwaterloo.ca/tsp/concorde.html
//First you have to build the Solver in any possible way and if you don't make a ros-package out of it you have to change
//the paths to the right ones. If it is a ros-package the ros::package::getpath() function will find the right path.
//The usage of the solver is: ./concorde [-see below-] [dat_file]
//Navigate to the build Solver and then ./TSP and type ./concorde -h for a short explanation.

//with a given distance matrix
std::vector<int> ConcordeTSPSolver::solveConcordeTSP(const Mat& path_length_matrix)
{
	// generate a unique filename
	timeval time;
	gettimeofday(&time, NULL);
	std::stringstream ss;
	ss << "_" << time.tv_sec << "_" << time.tv_usec;
	unique_file_identifier_ = ss.str();
	const std::string tsp_lib_filename = "TSPlib_file" + unique_file_identifier_ + ".atsp";
	const std::string tsp_order_filename = "TSP_order" + unique_file_identifier_ + ".atsp";

	std::vector<int> unsorted_order;
	std::cout << "finding optimal order" << std::endl;
	std::cout << "number of nodes: " << path_length_matrix.size() << std::endl;
	if (path_length_matrix.size() > 2) //check if the TSP has at least 3 nodes
	{
		//create the TSPlib file
		writeToFile(path_length_matrix, tsp_lib_filename, tsp_order_filename);
        // const std::string atsp_lib_filename = "ftv33.atsp";

        //use concorde to find optimal tour
		std::string bin_folder;
		while (bin_folder.length()==0)
		{
			const std::string temp_file = "temp_libconcorde_path" + unique_file_identifier_ + ".txt";
			try
			{
				std::string cmd = "rospack libs-only-L libconcorde_tsp_solver > " + temp_file;
				int result = system(cmd.c_str());
				std::ifstream file(temp_file.c_str(), std::ifstream::in);
				if (file.is_open())
				{
					file >> bin_folder;
					file.close();
				}
				std::cout << "bin_folder: " << bin_folder << std::endl;
				remove(temp_file.c_str());
				//bin_folder = ros::package::command("libs-only-L libconcorde_tsp_solver"); // this command crashes sometimes
				//bin_folder.erase(std::remove(bin_folder.begin(), bin_folder.end(), '\n'));
			}
			catch (...)
			{
				std::cout << "ConcordeTSPSolver::solveConcordeTSP: ERROR: ['rospack libs-only-L libconcorde_tsp_solver > '" << temp_file << "] failed. Trying again." << std::endl;
			}
		}
		std::string cmd = bin_folder + "/libconcorde_tsp_solver/concorde -o " + "./" + tsp_order_filename + " ./" + tsp_lib_filename;

		int result = system(cmd.c_str());
		assert(!result);

		//get order from saving file
		unsorted_order = readFromFile(tsp_order_filename);
	}
	else
	{
		for(int node = 0; node < path_length_matrix.size(); node++)
		{
			unsorted_order.push_back(node);
		}
	}
	// cleanup files
	remove(tsp_lib_filename.c_str());
	remove(tsp_order_filename.c_str());
	const std::string tsp_lib_sol_filename = "TSPlib_file" + unique_file_identifier_ + ".sol";
	remove(tsp_lib_sol_filename.c_str());
	const std::string tsp_lib_res_filename = "TSPlib_file" + unique_file_identifier_ + ".res";
	remove(tsp_lib_res_filename.c_str());
	std::cout << "finished TSP" << std::endl;

	// if there is an error, just set unsorted order to 1, 2, 3, ...
	if (unsorted_order.size() != path_length_matrix.size())
	{
		std::cout << "ConcordeTSPSolver::solveConcordeTSP: Warning: Optimized order invalid, taking standard order 1, 2, 3, ..." << std::endl;
		unsorted_order.clear();
		unsorted_order.resize(path_length_matrix.size());
		for (int i=0; i<path_length_matrix.size(); ++i)
			unsorted_order[i] = i;
	}

	return unsorted_order;
}