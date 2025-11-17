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

//Webots parameters
#define TIME_STEP 		32
#define MAX_ROBOTS 		100
#define MAX_PATROLS 	100

//Distance computation parameters
#define NUM_SAMPLES		50 //how many times we want to compute d_ij before we average it.

//ACS parameter
#define NUM_ITER		3	//how many iteration of ACS to perform
#define Q0				0.9 //probability to not explore (i.e. ants have 0.1 probability to explore)
#define BETA			1   //Importance of the distance relative to the pheromone level for the edges
#define RHO				0.1 //parameter for the local pheromone updates
#define ALPHA			0.1 //parameter for the global pheromone updates

//Debugging parameters
#define VERBOSE_TOURS	0   //whether to print the tours performed by each robot or not
#define VERBOSE_SAMPLES 1   //whether to print progress during sampling phase of the dist table computation
#define VERBOSE_PHER	0	//whether to print pher table after each iteration

static WbDeviceTag emitter, receiver;
static double L_NN= 0.0; //tour length produced by the nearest neighbor heuristic which will be computed

double uniform(double a, double b)
{
	double rng= (double)rand()/(double)RAND_MAX; //rng in [0,1]
	return a + rng*(b-a);
}

// Must provide true probabilities, i.e. sum(probs[0:n_patrols]) == 1.0
int custom_sampler(const double* probs, int n_patrols)
{
	double rng= (double)rand()/(double)(RAND_MAX);
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

double* at_edges(double* table, int p_id_i, int p_id_j, int n_patrols)
{
	return &table[p_id_i*n_patrols + p_id_j];
}

void add_city(Tour* tour, double* d_table, int p_id, int n_patrols)
{
	if (tour->num_patrols > 0){
		tour->length+= 1.0/(*at_edges(d_table, tour->path[tour->num_patrols-1], p_id, n_patrols));
	}
	tour->path[tour->num_patrols]= p_id;
	tour->num_patrols+= 1;
}

static bool get_info_of_robot(const char* name, double* x, double* y)
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

	//Accessing the position value
	const double* t = wb_supervisor_field_get_sf_vec3f(f); // [x, y, z]
	if (!t){
		return false; // Indicates that the pos wasn't succesfully loaded
	}

	*x = t[0];
	*y = t[1];
	return true;
}

// Helper: get 2D position (x,y) and color (r,g,b) of a node named DEF from the webot world
static bool get_info_of_node(const char* name, double* x, double* y, double* rgb) 
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

	//Accessing the position value
	const double* t = wb_supervisor_field_get_sf_vec3f(f); // [x, y, z]
	if (!t){
		return false; // Indicates that the pos wasn't succesfully loaded
	}

	*x = t[0];
	*y = t[1];

	//For the color
	WbFieldRef f_col = wb_supervisor_node_get_field(n, "children");
	if (!f_col){
		return false;
	}

	int count = wb_supervisor_field_get_count(f_col);
	for (int i = 0; i < count; ++i) {
		WbNodeRef child = wb_supervisor_field_get_mf_node(f_col, i);
		if (!child) continue;

		if (wb_supervisor_node_get_type(child) == WB_NODE_SHAPE) {
			WbFieldRef app_f = wb_supervisor_node_get_field(child, "appearance");
			if (!app_f) return false;

			WbNodeRef app = wb_supervisor_field_get_sf_node(app_f);
			if (!app) return false;

			WbFieldRef mat_f = wb_supervisor_node_get_field(app, "material");
			if (!mat_f) return false;

			WbNodeRef mat = wb_supervisor_field_get_sf_node(mat_f);
			if (!mat) return false;

			WbFieldRef diff_f = wb_supervisor_node_get_field(mat, "diffuseColor");
			if (!diff_f) return false;

			const double *c = wb_supervisor_field_get_sf_color(diff_f);
			rgb[0]= c[0];
			rgb[1]= c[1];
			rgb[2]= c[2];
			return true; // To indicate that the color was succesfully loaded
		}
	}

	return false; // To indicate that the color was not succesfully loaded
}

int load_info_robots(double* x_array, double* y_array, const char* format, int max_count)
{
	int counter= 0;
	char name[12];

	for (int i = 0; i < max_count; ++i) {
		snprintf(name, sizeof(name), format, i); // name then stores "PATROL0" if i=0, "PATROL1" if i=1, etc
		double x, y;

		if (!get_info_of_robot(name, &x, &y)) break; // this both checks if node with name "name" exists and stores its position in x and y

		x_array[i] = x;
		y_array[i] = y;

		++counter;
	}
	return counter;
}

int load_info_nodes(double* x_array, double* y_array, double* rgb_array, const char* format, int max_count)
{
	int counter= 0;
	char name[12];

	for (int i = 0; i < max_count; ++i) {
		snprintf(name, sizeof(name), format, i); // name then stores "PATROL0" if i=0, "PATROL1" if i=1, etc
		double x, y;
		double rgb[3];

		if (!get_info_of_node(name, &x, &y, rgb)) break; // this both checks if node with name "name" exists and stores its position in x and y

		x_array[i] = x;
		y_array[i] = y;

		rgb_array[i*3]= rgb[0];
		rgb_array[i*3 + 1]= rgb[1];
		rgb_array[i*3 + 2]= rgb[2];
		++counter;
		printf("[SUP] RGB couple of node %d: %.3f, %.3f, %.3f\n", i, rgb_array[i*3], rgb_array[i*3+1], rgb_array[i*3 + 2]);
	}
	return counter;
}

void broadcast_patrol_colors(int n_patrols, const double* rgbs)
{
	char send_buf[n_patrols*30];
	int written= snprintf(send_buf, sizeof(send_buf), "%d s", n_patrols);
	int offset= written;
	for (int i=0; i < n_patrols; ++i){
		written = snprintf(
        	send_buf + offset,
        	sizeof(send_buf) - offset,
        	"%.3f %.3f %.3f s",
        	rgbs[i*3],
        	rgbs[i*3 + 1],
        	rgbs[i*3 + 2]
    	);

		if (written < 0 || written >= sizeof(send_buf) - offset) {
			break;
    	}

    	offset += written;
	}
	wb_emitter_send(emitter, send_buf, strlen(send_buf) + 1);
}

void initialize(int* n_robots, int* n_patrols, double* patrol_x, double* patrol_y, double* r_x, double* r_y)
{
	wb_robot_init();
	srand(time(NULL)); // initialize the seed for rand


	emitter  = wb_robot_get_device("emitter");
	receiver = wb_robot_get_device("receiver");
	wb_receiver_enable(receiver, TIME_STEP);

	double rgbs[MAX_PATROLS*3];

	// --- Discover robots and patrol points from the webot world by their naming convention
	*n_patrols= load_info_nodes(patrol_x, patrol_y, rgbs, "PATROL%d", MAX_PATROLS);
	*n_robots= load_info_robots(r_x, r_y, "EPUCK%d", MAX_ROBOTS);

	broadcast_patrol_colors(*n_patrols, rgbs);

	printf("[SUP] Found %d robots and %d patrol points in the webot world.\n", *n_robots, *n_patrols);

	wb_robot_step(TIME_STEP);
}

//TODO change this to assign the next patrol properly based on distance, available targets and pheromone trails
int assign_patrol(double* d_table, double* pher_table, const Tour* tour, int n_patrols)
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
void step(int n_robots, int n_patrols, const double* patrol_x, const double* patrol_y,
		  double* d_table, double* pher_table, const Tour* r_tours)
{
	for (int r= 0; r < n_robots; ++r){ 
		int p= assign_patrol(d_table, pher_table, &r_tours[r], n_patrols);

		char msg[100];

		// Supervisor sends: <robot_id> <x> <y> <patrol_id>
		snprintf(msg, sizeof(msg), "%d %.3f %.3f %d", r, patrol_x[p], patrol_y[p], p);
		wb_emitter_send(emitter, msg, strlen(msg) + 1);
	}
}

Tour run_tour(int n_robots, int n_patrols, const double* patrol_x, const double* patrol_y,
			  double* d_table, double* pher_table, Tour* r_tours, const int* start_pos)
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
					
					int r_id;
					int p_id; // Current patrolled point
					int lp_id; // Last patrolled point
					double travel_time;
					//printf("%d\n", num_finished);
					if (sscanf(msg, "%d %d %d %lf", &r_id, &p_id, &lp_id, &travel_time) == 4){
				
						add_city(&r_tours[r_id], d_table, p_id, n_patrols);
						num_finished++;
					}
					wb_receiver_next_packet(receiver);
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
					
					int r_id;
					int p_id; // Current patrolled point
					int lp_id; // Last patrolled point
					double travel_time;
					
					if (sscanf(msg, "%d %d %d %lf", &r_id, &p_id, &lp_id, &travel_time) == 4){
						add_city(&r_tours[r_id], d_table, p_id, n_patrols); // adding the last step to the robots tour
						num_finished++;
					}
					wb_receiver_next_packet(receiver);
				}
			}
		}
		//local pheromone updates
		for (int r= 0; r < n_robots; ++r){
			int old_pos= r_tours[r].path[r_tours[r].num_patrols-2];
			int new_pos= r_tours[r].path[r_tours[r].num_patrols-1];
			double* to_modify= at_edges(pher_table, old_pos, new_pos, n_patrols);
			*to_modify= (1-RHO)*(*to_modify) + RHO*(1.0/(n_patrols*L_NN));
			*at_edges(pher_table, new_pos, old_pos, n_patrols) = *to_modify;
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
		double* to_modify= at_edges(pher_table, best_path[l], best_path[l+1], n_patrols);
		*to_modify= (1-ALPHA)*(*to_modify) + ALPHA*(1.0/min_length);
		*at_edges(pher_table, best_path[l+1], best_path[l], n_patrols)= *to_modify;
	}
	return best_tour;
}

double compute_L_NN(double* d_table, int n_patrols)
{
	double l_nn= 0.0;
	bool J_access[n_patrols]; //set of available cities for NN heuristic
	for (int i= 0; i < n_patrols; ++i){
		J_access[i]= true;
	}
	//we start the NN heuristic at node 0
	int current_pos= 0;
	J_access[0]= false;
	for (int i= 0; i < n_patrols-1; ++i){
		double max_inv_dist= 0.0;
		int next_pos= 0;
		for (int j= 0; j < n_patrols; ++j){
			if (J_access[j]){
				if (max_inv_dist < *at_edges(d_table, current_pos, j, n_patrols)){
					max_inv_dist= *at_edges(d_table, current_pos, j, n_patrols);
					next_pos= j;
				}
			}
		}
		J_access[next_pos] = false;
		current_pos= next_pos;
		l_nn+= 1.0/max_inv_dist;
	}
	l_nn+= 1.0 / *at_edges(d_table, current_pos, 0, n_patrols);
	return l_nn;
}

void fill_distance_table(double* d_table, int n_patrols, int n_robots,
                         const double* patrol_x, const double* patrol_y,
                         const int* start_pos){
	// How many arrivals we want to record in total
	const int MAX_SAMPLES = NUM_SAMPLES;
	printf("[SUP] Starting the computation of the distance table using %d samples.\n", MAX_SAMPLES);

	// Accumulated travel times and counts for each (i,j)
	double sum_times[n_patrols*n_patrols];
	double counts[n_patrols*n_patrols];

	// Initialize accumulators
	for (int i = 0; i < n_patrols; ++i) {
		for (int j = 0; j < n_patrols; ++j) {
			*at_edges(sum_times, i, j, n_patrols) = 0.0;
			*at_edges(counts, i, j, n_patrols) = 0;
		}
	}

	// For each robot, send it to an initial random patrol that is NOT its start_pos
	for (int r = 0; r < n_robots; ++r) {
		int from = start_pos[r];
		int to;

		do {
			to = rand() % n_patrols;
		} while (to == from);  // ensure destination != current node

		char msg[100];
		snprintf(msg, sizeof(msg), "%d %.3f %.3f %d", r, patrol_x[to], patrol_y[to], to);
		wb_emitter_send(emitter, msg, strlen(msg) + 1);
	}

	int samples = 0;

	// Main loop: wait for robots to arrive and keep sending them to new random patrols
	while (wb_robot_step(TIME_STEP) != -1 && samples < MAX_SAMPLES) {
		while (wb_receiver_get_queue_length(receiver) > 0 && samples < MAX_SAMPLES) {
			const char *msg = wb_receiver_get_data(receiver);

			int r_id;
			int p_id;   // current patrol reached
			int lp_id;  // last patrol
			double travel_time;

			// Robot sends: "<r_id> <p_id> <lp_id> <travel_time>"
			if (sscanf(msg, "%d %d %d %lf", &r_id, &p_id, &lp_id, &travel_time) == 4) {
			// Sanity checks
			if (r_id >= 0 && r_id < n_robots &&
				p_id >= 0 && p_id < n_patrols &&
				lp_id >= 0 && lp_id < n_patrols &&
				p_id != lp_id) {

				// Accumulate time for edge lp_id -> p_id
				*at_edges(sum_times, p_id, lp_id, n_patrols) += travel_time;
				*at_edges(counts, p_id, lp_id, n_patrols) += 1;

				// If you want symmetric distances, also accumulate the reverse:
				*at_edges(sum_times, lp_id, p_id, n_patrols) += travel_time;
				*at_edges(counts, lp_id, p_id, n_patrols) += 1;

				samples++;
				if (VERBOSE_SAMPLES){
					if (samples % 10 == 0){
						printf("[SUP] %d samples have been collected.\n", samples);
					}
				}
			}

			// Now send this robot to a NEW random patrol != current p_id
			int next_p;
			do {
				next_p = rand() % n_patrols;
			} while (next_p == p_id);  // don't send to the same node it is on

			char out[100];
			snprintf(out, sizeof(out), "%d %.3f %.3f %d", r_id, patrol_x[next_p], patrol_y[next_p], next_p);
			wb_emitter_send(emitter, out, strlen(out) + 1);
			}

			wb_receiver_next_packet(receiver);
		}
	}

	// Convert accumulated times into averaged "distances" for d_table
	// and invert them to follow the 1/dist convention (now 1/time).
	for (int i = 0; i < n_patrols; ++i) {
		for (int j = 0; j < n_patrols; ++j) {
			double count_edge= *at_edges(counts, i, j, n_patrols);
			if (count_edge > 0) {
				double avg_time = *at_edges(sum_times, i, j, n_patrols) / count_edge;
				*at_edges(d_table, i, j , n_patrols) = 1.0 / avg_time;
			} else {
				// No samples for this edge, if i==j, this is fine, else fallback is to assign 0
				if (i != j){
					printf("[SUP] Edge %d->%d was never sampled for distance computation.\n", i, j);
					printf("[SUP] Setting its inverted distance to 0 as a fallback.\n");
				}
				*at_edges(d_table, i, j , n_patrols) = 0.0;
			}
		}
	}

	printf("[SUP] Distance table filled using %d samples.\n", samples);

	// Sending all the robots back to their starting position
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
			
			int r_id;
			int p_id; // Current patrolled point
			int lp_id; // Last patrolled point
			double travel_time;
			
			if (sscanf(msg, "%d %d %d %lf", &r_id, &p_id, &lp_id, &travel_time) == 4){
				num_finished++;
			}
			wb_receiver_next_packet(receiver);
		}
	}
}

void init_edge_tables(double* d_table, double* pher_table, const double* patrol_x, 
					  const double* patrol_y, const int* start_pos,int n_robots, int n_patrols)
{
	fill_distance_table(d_table, n_patrols, n_robots, patrol_x, patrol_y, start_pos);

	L_NN= compute_L_NN(d_table, n_patrols); //computing L_NN using Nearest Neighbor heuristic
	printf("[SUP] The tour length obtained with NN heuristic is: %.3f\n", L_NN);

	for (int i= 0; i < n_patrols*n_patrols; ++i){
		pher_table[i]= 1.0/(n_patrols*L_NN);
	}
}

bool* at_nodes(bool* acces_tab, int r_id, int p_id, int n_patrols)
{
	return &acces_tab[r_id*n_patrols + p_id];
}

void reset_robot_tours(Tour* r_tours, double* d_table, const int* start_pos, int n_robots, int n_patrols)
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

void compute_start_pos(const double* p_x, const double* p_y, const double* r_x, const double* r_y,
					   int* start_pos, int n_robots, int n_patrols)
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

void print_edge_table(double* tab, int n_patrols)
{
	for (int i=0; i < n_patrols; ++i){
		for (int j=0; j < n_patrols; ++j){
			printf("%.3f ", *at_edges(tab, i, j, n_patrols));
		}
		printf("\n");
	}
}

double compute_length_path(double* d_table, const int* path, int n_patrols)
{
	double retour=0;
	for (int i= 0; i < n_patrols; ++i){
		retour+= 1.0/(*at_edges(d_table, path[i], path[i+1], n_patrols));
	}
	return retour;
}

double brute_force_5_nodes_solution(double* d_table, int n_patrols, int* best_path)
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

void terminate_robots_controller(){
	// Supervisor sends: "STOP"
	char msg[10];
	snprintf(msg, sizeof(msg), "%s", "STOP");
	wb_emitter_send(emitter, msg, strlen(msg) + 1);
}

int main() 
{
	int n_robots, n_patrols;
	double patrol_x[MAX_PATROLS], patrol_y[MAX_PATROLS]; // stores the x and y positions of the to be discovered patrols
	double robots_x[MAX_ROBOTS], robots_y[MAX_ROBOTS];

	initialize(&n_robots, &n_patrols, patrol_x, patrol_y, robots_x, robots_y);

	double d_table[n_patrols*n_patrols]; //distance table (for optimization purposes, each dist is assumed to be stored inverted i.e. 1/dist)
	double pher_table[n_patrols*n_patrols]; //pheromone table

	// list of the starting nodes for each robot (will not change during tours since tours must be cyclic)
	int start_pos[n_robots];
	compute_start_pos(patrol_x, patrol_y, robots_x, robots_y, start_pos, n_robots, n_patrols);
           
	init_edge_tables(d_table, pher_table, patrol_x, patrol_y, start_pos, n_robots, n_patrols);

	printf("[SUP] Inverted distance table:\n");
	print_edge_table(d_table, n_patrols);
	printf("\n");
	printf("[SUP] Initial pheromone table:\n");
	print_edge_table(pher_table, n_patrols);

	Tour robots_tours[n_robots];

	Tour overall_best_tour;
	overall_best_tour.length= DBL_MAX;
	overall_best_tour.num_patrols= 0;
	overall_best_tour.path[0]= 0;
	int best_iter= 0;

	for (int i= 0; i < NUM_ITER; ++i){
		printf("[SUP] Starting the tour %d.\n", i+1);

		reset_robot_tours(robots_tours, d_table, start_pos, n_robots, n_patrols);

		Tour best_tour_of_run= run_tour(n_robots, n_patrols, patrol_x, patrol_y, d_table, pher_table, robots_tours, start_pos);

		if (overall_best_tour.length > best_tour_of_run.length){
			overall_best_tour= best_tour_of_run;
			best_iter= i;
		}

		printf("[SUP] The tour %d is completed.\n", i+1);

		if (VERBOSE_TOURS){
			for (int r=0; r < n_robots; ++r){
				print_tour(&robots_tours[r], r);
			}
		}else{
			double average_tour_length= 0.0;
			for (int r=0; r < n_robots; ++r){
				average_tour_length+= robots_tours[r].length;
			}
			average_tour_length/= n_robots;
			printf("[SUP] Average tour length during this tour: %.3f\n", average_tour_length);
		}

		if (VERBOSE_PHER){
			printf("[SUP] Updated pheromone table:\n");
			print_edge_table(pher_table, n_patrols);
		}
	}
	terminate_robots_controller();

	printf("[SUP] The best tour was found in iteration %d. This tour is: %d", best_iter + 1, overall_best_tour.path[0]);
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


