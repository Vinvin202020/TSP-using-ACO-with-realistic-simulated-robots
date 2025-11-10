// controllers/supervisor/supervisor.c
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include <float.h>
#include <string.h>

#define TIME_STEP 		32
#define MAX_ROBOTS 		10
#define MAX_PATROLS 	10
#define NUM_ITER		15
#define L_NN			10  //tour length produced by the nearest neighbor heuristic TODO change?
#define Q0				0.9 //probability to not explore (i.e. ants have 0.1 probability to explore)
#define BETA			1   //Importance of the distance relative to the pheromone level for the edges
#define VERBOSE_TOURS	1   //whether to print the tours performed by each robot or not
#define RHO				0.1 //parameter for the local pheromone updates
#define ALPHA			0.1 //parameter for the global pheromone updates

static WbDeviceTag emitter, receiver;

double uniform(double a, double b)
{
	double rng= (double)rand()/(double)RAND_MAX; //rng in [0,1]
	return a + rng*(b-a);
}

// Must provide true probabilities, i.e. sum(probs[0:n_patrols]) == 1.0
int custom_sampler(const double* probs, int n_patrols)
{
	double rng= (double)rand()/(double)(RAND_MAX + 1);
	//+ 1 here to avoid the possibility for rng to be equal to one,
	//because if rng is equal to 1, floating point errors could lead 
	//sum to never reach 1 after the for loop which would wrek havoc
	double sum= 0.0;
	int last_nonzero= -1;
	for (int i= 0; i < n_patrols; ++i){
		if (probs[i]> 0.0){
			last_nonzero= i;
		}
		sum+= probs[i];
		if (sum >= rng){
			return i;
		}
	}
	return last_nonzero; //fallback
}

typedef struct {
	double length; //store the length (sum of distances) in the tour
	int num_patrols; //stores the number of cities in the tour
	int path[MAX_PATROLS + 1];
} Tour;

const double* at_edges(const double* table, int p_id_i, int p_id_j, int n_patrols)
{
	return &table[p_id_i*n_patrols + p_id_j];
}

void add_city(Tour* tour, const double* d_table, int p_id, int n_patrols)
{
	if (tour->num_patrols > 0){
		tour->length+= 1.0/(*at_edges(d_table, tour->path[tour->num_patrols-1], p_id, n_patrols));
	}
	tour->path[tour->num_patrols]= p_id;
	tour->num_patrols+= 1;
}

// Helper: get 2D position (x,y) of a node named DEF from the webot world
static bool get_pos_of_node(const char* name, double* x, double* y) 
{
	// Attempting to access the node with name def in the webot world
	WbNodeRef n = wb_supervisor_node_get_from_def(name);
	if (!n){
		return false; // To indicates that we have reached a node that does not exists in the webot world
	}
	// Access the translation field of the node stored in n
	WbFieldRef f = wb_supervisor_node_get_field(n, "translation");
	if (!f){
		return false;
	}

	const double* t = wb_supervisor_field_get_sf_vec3f(f); // [x, y, z]
	if (!t){
		return false;
	}

	*x = t[0];
	*y = t[1];

	return true; // To indicate that the position of the node was succesfully loaded
}

int load_pos(double* x_array, double* y_array, const char* format, int max_count)
{
	int n= 0;
	char name[12];

	for (int i = 0; i < max_count; ++i) {
		snprintf(name, sizeof(name), format, i); // name then stores "PATROL0" if i=0, "PATROL1" if i=1, etc
		double x, y;

		if (!get_pos_of_node(name, &x, &y)) break; // this both checks if node with name "name" exists and stores its position in x and y

		x_array[n] = x;
		y_array[n] = y;

		n++;
	}
	return n;
}

void initialize(int* n_robots, int* n_patrols, double* patrol_x, double* patrol_y, double* r_x, double* r_y)
{
	wb_robot_init();
	srand(time(NULL)); // initialize the seed for rand


	emitter  = wb_robot_get_device("emitter");
	receiver = wb_robot_get_device("receiver");
	wb_receiver_enable(receiver, TIME_STEP);

	// --- Discover robots and patrol points from the webot world by their naming convention
	*n_patrols= load_pos(patrol_x, patrol_y, "PATROL%d", MAX_PATROLS);
	*n_robots= load_pos(r_x, r_y, "EPUCK%d", MAX_ROBOTS);

	printf("[SUP] Found %d robots and %d patrol points in the webot world.\n", *n_robots, *n_patrols);

	wb_robot_step(TIME_STEP);
}

//TODO change this to assign the next patrol properly based on distance, available targets and pheromone trails
int assign_patrol(const double* d_table, const double* pher_table, const Tour* tour, int n_patrols)
{
	double q= uniform(0, 1);
	bool J_access[n_patrols]; //set of available cities for the ant
	for (int i= 0; i < n_patrols; ++i){
		J_access[i]= true;
	}
	for (int i= 0; i < tour->num_patrols; ++i){
		J_access[tour->path[i]]= false;
	}

	int current_city= tour->path[tour->num_patrols-1]; //current position of the ant

	if (q <= Q0){ //exploitation with proba Q0 (no randomness)
		int amax= 0;
		double max= -1.0;
		for (int i= 0; i < n_patrols; ++i){
			if (J_access[i]){
				double tmp_mem= *at_edges(pher_table, current_city, i, n_patrols)* pow(*at_edges(d_table, current_city, i, n_patrols),BETA);
				if (max < tmp_mem){
					max= tmp_mem;
					amax= i;
				}
			}
		}
		return amax;
	}else{ //random exploration with weighted probabilities
		double probas[n_patrols];
		double numerator= 0.0;
		for (int i= 0; i < n_patrols; ++i){
			if (J_access[i]){
				numerator+= *at_edges(pher_table, current_city, i, n_patrols)* pow(*at_edges(d_table, current_city, i, n_patrols), BETA);
			}
		}
		for (int i= 0; i < n_patrols; ++i){
			if (J_access[i]){
				probas[i]= *at_edges(pher_table, current_city, i, n_patrols)* pow(*at_edges(d_table, current_city, i, n_patrols), BETA)/numerator;
			}else{
				probas[i]= 0.0;
			}
		}
		return custom_sampler(probas, n_patrols);
	}
}

//Function that assigns a patrol to each robot and send them the required information
void step(int n_robots, int n_patrols, const double* patrol_x, const double* patrol_y, const double* d_table, const double* pher_table, const Tour* r_tours)
{
	for (int r= 0; r < n_robots; ++r){ 
		int p= assign_patrol(d_table, pher_table, &r_tours[r], n_patrols);

		char msg[100];

		// Supervisor sends: <robot_id> <x> <y> <patrol_id>
		snprintf(msg, sizeof(msg), "%d %.3f %.3f %d", r, patrol_x[p], patrol_y[p], p);
		wb_emitter_send(emitter, msg, strlen(msg) + 1);
	}
}

Tour run_tour(int n_robots, int n_patrols, const double* patrol_x, const double* patrol_y, const double* d_table, double* pher_table, Tour* r_tours, const int* start_pos)
{
	Tour best_tour;
	//Each robots must move n_patrols-1 times to create a tour (assuming they already start on a node)
	for (int p= 0; p < n_patrols; ++p){
		if (p < n_patrols - 1){
			step(n_robots, n_patrols, patrol_x, patrol_y, d_table, pher_table, r_tours);
			//loop to wait for the robots to complete their patrols
			int num_finished= 0;
			while ((wb_robot_step(TIME_STEP) != -1) && (num_finished < n_robots)){
				while (wb_receiver_get_queue_length(receiver) > 0){
					const char *msg = wb_receiver_get_data(receiver);
					wb_receiver_next_packet(receiver);
					int r_id;
					int p_id;
					if (sscanf(msg, "%d %d", &r_id, &p_id) == 2){
						add_city(&r_tours[r_id], d_table, p_id, n_patrols);
						num_finished++;
					}
				}
			}
		}else{
			//Now robots complete their tour by going back to their starting position
			for (int r= 0; r < n_robots; ++r){
				int p= start_pos[r];

				char msg[100];

				// Supervisor sends: <robot_id> <x> <y> <patrol_id>
				snprintf(msg, sizeof(msg), "%d %.3f %.3f %d", r, patrol_x[p], patrol_y[p], p);
				wb_emitter_send(emitter, msg, strlen(msg) + 1);
			}

			int num_finished= 0; // Wait for all the robots to complete their tasks (i.e. reach their starting position)
			while ((wb_robot_step(TIME_STEP) != -1) && (num_finished < n_robots)){
				while (wb_receiver_get_queue_length(receiver) > 0){
					const char *msg = wb_receiver_get_data(receiver);
					wb_receiver_next_packet(receiver);
					int r_id;
					int p_id;
					if (sscanf(msg, "%d %d", &r_id, &p_id) == 2){
						add_city(&r_tours[r_id], d_table, p_id, n_patrols); // adding the last step to the robots tour
						num_finished++;
					}
				}
			}
		}
		//local pheromone updates
		for (int r= 0; r < n_robots; ++r){
			int old_pos= r_tours[r].path[r_tours[r].num_patrols-2];
			int new_pos= r_tours[r].path[r_tours[r].num_patrols-1];
			double* to_modify= (double*) at_edges(pher_table, old_pos, new_pos, n_patrols);
			*to_modify= (1-RHO)*(*to_modify) + RHO*(1.0/(n_patrols*L_NN));
			*(double*)at_edges(pher_table, new_pos, old_pos, n_patrols) = *to_modify;
		}
	}
	//global pheromone updates
	double min_length= DBL_MAX;
	int best_id= 0;
	for (int r= 0; r < n_robots; ++r){
		if (min_length > r_tours[r].length){
			min_length= r_tours[r].length;
			best_id= r;
		}
	}
	best_tour= r_tours[best_id];
	int* best_path= best_tour.path;
	for (int l= 0; l < n_patrols; ++l){
		double* to_modify= (double*) at_edges(pher_table, best_path[l], best_path[l+1], n_patrols);
		*to_modify= (1-ALPHA)*(*to_modify) + ALPHA*(1.0/min_length);
		*(double*)at_edges(pher_table, best_path[l+1], best_path[l], n_patrols)= *to_modify;
	}
	return best_tour;
}

void init_edge_tables(double* d_table, double* pher_table, int n_patrols)
{
	//TODO: Change this when Etienne has found a way to compute the d_table
	//REMAINDER: d_ij= d_table[i*n_patrols + j]= 1/dist (must store inverted distances)
	for (int i= 0; i < n_patrols; ++i){
		for (int j= 0; j <= i; ++j){
			d_table[i*n_patrols + j]= (double)rand()/(double)RAND_MAX;
			d_table[j*n_patrols + i]= d_table[i*n_patrols + j];
		}
	}
	for (int i= 0; i < n_patrols*n_patrols; ++i){
		pher_table[i]= 1.0/(n_patrols*L_NN);
	}
}

bool* at_nodes(bool* acces_tab, int r_id, int p_id, int n_patrols)
{
	return &acces_tab[r_id*n_patrols + p_id];
}

void reset_robot_tours(Tour* r_tours, const double* d_table, const int* start_pos, int n_robots, int n_patrols)
{
	for (int r= 0; r < n_robots; ++r){
		r_tours[r].num_patrols= 0;
		r_tours[r].length= 0;
		add_city(&r_tours[r], d_table, start_pos[r], n_patrols);
	}

}

double eucl_dist(double x_1, double y_1, double x_2, double y_2)
{
	double dx= x_1 - x_2;
	double dy= y_1 - y_2;
	return sqrt(dx*dx + dy*dy);
}

void compute_start_pos(const double* p_x, const double* p_y, const double* r_x, const double* r_y, int* start_pos, int n_robots, int n_patrols)
{
	for (int r= 0; r < n_robots; ++r){
		double min_dist= DBL_MAX;
		int min_p_id= 0;
		for (int p= 0; p < n_patrols; ++p){
			double dist_tmp= eucl_dist(r_x[r], r_y[r], p_x[p], p_y[p]);
			if (min_dist > dist_tmp){
				min_dist= dist_tmp;
				min_p_id= p;
			}
		}
		start_pos[r]= min_p_id;
	}
}

void print_tour(const Tour* tour, int r_id)
{
	printf("[SUP] Robot %d performed the folowing tour: %d", r_id, tour->path[0]);
	for (int i= 1; i < tour->num_patrols; ++i){
		printf("->%d", tour->path[i]);
	}
	printf(" of length: %.3f.\n", tour->length);
}

void print_edge_table(const double* tab, int n_patrols)
{
	for (int i=0; i < n_patrols; ++i){
		for (int j=0; j < n_patrols; ++j){
			printf("%.3f ", *at_edges(tab, i, j, n_patrols));
		}
		printf("\n");
	}
}

double compute_length_path(const double* d_table, const int* path, int n_patrols)
{
	double retour=0;
	for (int i= 0; i < n_patrols; ++i){
		retour+= 1.0/(*at_edges(d_table, path[i], path[i+1], n_patrols));
	}
	return retour;
}

double brute_force_5_nodes_solution(const double* d_table, int n_patrols, int* best_path)
{
	int test_path[n_patrols + 1];
	test_path[0]= 0;
	test_path[n_patrols]= 0;
    double best_length = DBL_MAX;
	for (int i= 1; i < n_patrols; ++i){
		test_path[1]= i;
		for (int j= 1; j < n_patrols; ++j){
			if (i!=j){
				test_path[2]= j;
				for (int k=1; k < n_patrols; ++k){
					if ((k!=i) && (k!=j)){
						test_path[3]= k;
						for (int l= 1; l < n_patrols; ++l){
							if ((l!=i) && (l!=j) && (l!=k)){
								test_path[4]= l;
								double test_length= compute_length_path(d_table, test_path, n_patrols);
								if (test_length < best_length){
									best_length= test_length;
									memcpy(best_path, test_path, (n_patrols + 1) * sizeof(int));
								}
							}
						}
					}
				}
			}
		}
	}
    return best_length;
}
int main() 
{
	int n_robots, n_patrols;
	double patrol_x[MAX_PATROLS], patrol_y[MAX_PATROLS]; // stores the x and y positions of the to be discovered patrols
	double robots_x[MAX_ROBOTS], robots_y[MAX_ROBOTS];

	initialize(&n_robots, &n_patrols, patrol_x, patrol_y, robots_x, robots_y);

	double d_table[n_patrols*n_patrols]; //distance table (for optimization purposes, each dist is assumed to be stored inverted i.e. 1/dist)
	double pher_table[n_patrols*n_patrols]; //pheromone table

	init_edge_tables(d_table, pher_table, n_patrols);

	printf("[SUP] Distance table:\n");
	print_edge_table(d_table, n_patrols);

	printf("[SUP] Initial pheromone table:\n");
	print_edge_table(pher_table, n_patrols);

	// list of the starting nodes for each robot (will not change during tours since tours must be cyclic)
	int start_pos[n_robots];
	compute_start_pos(patrol_x, patrol_y, robots_x, robots_y, start_pos, n_robots, n_patrols);

	// list of the accessibility of the cities for each robot, 
	// evolves during each tour and is then reset at the end of a tour
	Tour robots_tours[n_robots];

	Tour overall_best_tour;
	overall_best_tour.length= DBL_MAX;
	overall_best_tour.num_patrols= 0;
	overall_best_tour.path[0]= 0;

	for (int i= 0; i < NUM_ITER; ++i){
		printf("[SUP] Starting the tour %d.\n", i+1);

		reset_robot_tours(robots_tours, d_table, start_pos, n_robots, n_patrols);

		Tour best_tour_of_run= run_tour(n_robots, n_patrols, patrol_x, patrol_y, d_table, pher_table, robots_tours, start_pos);

		if (overall_best_tour.length > best_tour_of_run.length){
			overall_best_tour= best_tour_of_run;
		}

		printf("[SUP] The tour %d is completed.\n", i+1);

		if (VERBOSE_TOURS){
			for (int r=0; r < n_robots; ++r){
				print_tour(&robots_tours[r], r);
			}
		}

		printf("[SUP] Updated pheromone table:\n");
		print_edge_table(pher_table, n_patrols);
	}

	printf("[SUP] The best tour found after %d iteration is: %d", NUM_ITER, overall_best_tour.path[0]);
	for (int i= 1; i < overall_best_tour.num_patrols; ++i){
		printf("->%d", overall_best_tour.path[i]);
	}
	printf(" of length: %.3f.\n", overall_best_tour.length);

	if (n_patrols == 5){
		int optimal_path[n_patrols + 1];
		double optimal_length= brute_force_5_nodes_solution(d_table, n_patrols, optimal_path);
		printf("[SUP] The optimal tour has length: %.3f\n", optimal_length);
		printf("[SUP] The optimal path is: %d", optimal_path[0]);
		for (int i= 1; i < overall_best_tour.num_patrols; ++i){
			printf("->%d", optimal_path[i]);
		}
		printf("\n");
	}

	wb_robot_cleanup();
	return 0;
}


