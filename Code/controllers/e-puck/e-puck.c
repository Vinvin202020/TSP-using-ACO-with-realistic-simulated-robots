// controllers/e-puck/e-puck.c
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/camera.h>

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#define TIME_STEP   32
#define NB_SENSORS  8
#define MAX_V       6.28
#define MAX_SENS    4095.0
// normaliser les IR
#define ARRIVE_EPS  0.2
// tolerance pour l'arrivee. a enlever apres qd on detectera avec les couleurs
#define VERBOSE_BOTS 0

static WbDeviceTag left_motor, right_motor;
static WbDeviceTag ps[NB_SENSORS];
static WbDeviceTag gps, imu;
static WbDeviceTag emitter, receiver;
static WbDeviceTag cam;

// robot info
static int my_id = -1;
static int target_patrol = -1;
static int last_patrol = -1;
static bool has_target = 0;
static double tx = 0.0, ty = 0.0; // CIBLE dans le plan X–Y
static double start_time = 0;
static double end_time = 0;
static double travel_time = 0;
static bool in_progress= true;

// Color detection
static int cam_w, cam_h;
static int color_hits= 0;
static const double TOL_GEN= 10.0;
static int consistency= 5;

// Braitenberg
static const double BASE_SPEED = 5;
static const double AVOID_GAIN = 1000.0;
static const double L_WEIGHT[NB_SENSORS] = { -1.0, -0.8, -0.5,  0.0,  0.0,  0.5,  0.8,  1.0 };
static const double R_WEIGHT[NB_SENSORS] = {  1.0,  0.8,  0.5,  0.0,  0.0, -0.5, -0.8, -1.0 };

// Go-to-goal (dans le plan X–Y)
static const double K_TURN  = 4.0;
static const double K_FWD   = 2.0;
static const double FWD_CAP = 0.3;

static double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
static void set_speed(double vl, double vr) {
  wb_motor_set_velocity(left_motor,  clamp(vl, -MAX_V, MAX_V));
  wb_motor_set_velocity(right_motor, clamp(vr, -MAX_V, MAX_V));
}

//function that parse the supervisor message and stores the usefull information for the robot
static void parse_supervisor_msg(const char *msg) {
  // "<id> <x> <y> <patrol_id>"
  int id, pid; double x, y;
  if (sscanf(msg, "%d %lf %lf %d", &id, &x, &y, &pid) == 4) {
    if (id == my_id) {
      tx = x; ty = y; target_patrol = pid; has_target = true;
	  start_time = wb_robot_get_time();
	  if (VERBOSE_BOTS){
      	printf("[R%d] Target set: (%.3f, %.3f), patrol=%d\n", my_id, tx, ty, target_patrol);
	  }
    }
  }else{
	if (strcmp(msg, "STOP") == 0){
		in_progress= false;
	}
  }
}

// --- ID unique depuis le champ "name", p.ex. "EPUCK3" -> 3
int parse_id_from_name(const char *nm) 
{
	// cherche les derniers chiffres dans le nom
	int id = -1, tmp = -1;
	for (const char *p = nm; *p; ++p) {
		if (*p >= '0' && *p <= '9') {
			if (tmp < 0) tmp = 0;
			tmp = tmp * 10 + (*p - '0');
		} else if (tmp >= 0) { id = tmp; tmp = -1; }
	}
	if (tmp >= 0) id = tmp;
	return id;
}

void camera_init() {
	cam = wb_robot_get_device("camera");
	wb_camera_enable(cam, TIME_STEP);
	cam_w = wb_camera_get_width(cam);
	cam_h = wb_camera_get_height(cam);
}

void rgb_to_hsv(double r, double g, double b, double *h, double *s, double *v) {
  double max = fmax(r, fmax(g,b)), min = fmin(r, fmin(g,b));
  double d = max - min;
  *v = max;
  *s = (max <= 1e-6) ? 0.0 : d / max;
  double hh;
  if (d < 1e-6) hh = 0.0;
  else if (max == r) hh = fmod(((g - b) / d), 6.0);
  else if (max == g) hh = ((b - r) / d) + 2.0;
  else            	hh = ((r - g) / d) + 4.0;
  hh *= 60.0;
  if (hh < 0.0) hh += 360.0;
  *h = hh;
}

// calcule H,S,V moyens sur la fenêtre ROI (bas-centre recommandé)
void roi_hsv(double *H, double *S, double *V) {
	const unsigned char *img = wb_camera_get_image(cam);
	int cx = cam_w/2, cy = (3*cam_h)/4, half = 3; // fenetre a observer
	double Hsum=0, Ssum=0, Vsum=0; int count=1;  // faux de 1 mais balec
	//const double RGB_SUM_MIN = 0.02; // ~ 0.02 par canal (sur [0..1])
	const double V_BLACK     = 0.08; // pixels trop sombres => skip
	
	for (int y=cy-half; y<=cy+half; ++y)
		for (int x=cx-half; x<=cx+half; ++x) {
			double r = wb_camera_image_get_red(img,   cam_w, x,y)/255.0;
			double g = wb_camera_image_get_green(img, cam_w, x,y)/255.0;
			double b = wb_camera_image_get_blue(img,  cam_w, x,y)/255.0;
			// 1) Rejette les quasi-noirs sans calculer HSV
				// if ((r + g + b) < RGB_SUM_MIN) continue;
				
			double h,s,v; rgb_to_hsv(r,g,b,&h,&s,&v);
			// 2) Rejette si V (luminosité) est trop bas
			if (v < V_BLACK) continue;
			//if (h>10 || v>10|| s>10){
			Hsum += h; Ssum += s; Vsum += v; count++;
		}
	*H = Hsum / count; *S = Ssum / count; *V = Vsum / count;
}

void read_rgb_list(double* rgbs, const char* msg, int n_patrols)
{
	int count= 0;
	int slider= 0;
	while (count < n_patrols){
		if (msg[slider] == 's'){
			int consumed;
			if (sscanf(&msg[slider], "s%lf %lf %lf %n",
				   &rgbs[3*count], &rgbs[3*count + 1],
				   &rgbs[3*count + 2], &consumed) == 3){
				++count;
				slider+= consumed;
			}
		}else{
			++slider;
		}
	}
	if (my_id == 0){
		for (int i= 0; i < n_patrols; ++i){
			printf("[ROB] RGB couple of node %d: %.3f, %.3f, %.3f\n", i, rgbs[i*3], rgbs[i*3+1], rgbs[i*3 + 2]);
		}
	}
}

double hue_dist(double h1, double h2){
	double d = fabs(h1 - h2);
	if (d > 180.0) d = 360.0 - d;
	return d;
}

bool color_arrived(const double* rgbs) {
	// 1) lire la couleur cible en HSV
	double hr, sr, vr; // hsv cible
	double r = rgbs[target_patrol*3];
	double g = rgbs[target_patrol*3 + 1];
	double b = rgbs[target_patrol*3 + 2];

	rgb_to_hsv(r,g,b,&hr,&sr,&vr);

	// lire HSV moyen dans la ROI (i.e. HSV de la caméra du robot)
	double h,s,v; 
	roi_hsv(&h,&s,&v);

	// stabilité temporelle
	if (hue_dist(h, hr) < TOL_GEN) {
		color_hits++;
		return (color_hits >= consistency); // 8 cycles stables ~0.5 s si TIME_STEP=64
	} else {
		color_hits = 0;
		return false;
	}
}

double* initialize()
{
	wb_robot_init();
	camera_init();
  
  	const char *nm = wb_robot_get_name();
  	my_id = parse_id_from_name(nm);
  	if (my_id < 0) {
    	fprintf(stderr, "[%s] ERROR: robot name must end with an integer, e.g. EPUCK0\n", nm);
    	my_id = 0; // fallback, mais idéalement corrige le name dans le monde
  	}

  	// Moteurs
  	left_motor  = wb_robot_get_device("left wheel motor");
  	right_motor = wb_robot_get_device("right wheel motor");
  	wb_motor_set_position(left_motor, INFINITY);
  	wb_motor_set_position(right_motor, INFINITY);
 	wb_motor_set_velocity(left_motor, 0.0);
  	wb_motor_set_velocity(right_motor, 0.0);

	// IR
	char name[8] = "ps0";
	for (int i = 0; i < NB_SENSORS; ++i) {
		ps[i] = wb_robot_get_device(name);
		wb_distance_sensor_enable(ps[i], TIME_STEP);
		name[2]++; // ps0->ps1...
	}

	// GPS + IMU
	gps = wb_robot_get_device("gps");
	imu = wb_robot_get_device("inertial unit");
	wb_gps_enable(gps, TIME_STEP);
	wb_inertial_unit_enable(imu, TIME_STEP);

	// Radio
	emitter  = wb_robot_get_device("emitter");
	receiver = wb_robot_get_device("receiver");
	wb_receiver_enable(receiver, TIME_STEP);

	// Sanity logs (une fois)
	if (!gps) fprintf(stderr, "[R?][ERR] GPS 'gps' not found\n");
	if (!imu) fprintf(stderr, "[R?][ERR] InertialUnit 'inertial unit' not found\n");
	for (int i = 0; i < NB_SENSORS; ++i)
		if (!ps[i]) fprintf(stderr, "[R?][ERR] DistSensor ps%d not found\n", i);

	while ((wb_robot_step(TIME_STEP)!=-1) && (wb_receiver_get_queue_length(receiver) == 0)){
		//waiting for supervisor to send info
	}
	const char* msg= wb_receiver_get_data(receiver);
	int n_patrols;
	if (sscanf(msg, "%d s ", &n_patrols)!=1){
		printf("[ROB] Robot %d failed to read n_patrols.", n_patrols);
	}
	double* rgbs= malloc(3*n_patrols*sizeof(double));
	read_rgb_list(rgbs, msg, n_patrols);

	wb_robot_step(TIME_STEP);

	return rgbs;
}

void receive_patrol()
{
	while (wb_receiver_get_queue_length(receiver) > 0) {
		const char *data = wb_receiver_get_data(receiver);
		parse_supervisor_msg(data);
		wb_receiver_next_packet(receiver);
	}
}

void braitenberg_dodging(double* vL, double* vR)
{
	const double *p = wb_gps_get_values(gps);                // [x, y, z]
	const double *r = wb_inertial_unit_get_roll_pitch_yaw(imu);
	double x = p[0], y = p[1]; 
	double dx = tx - x, dy = ty - y;
	double dist = sqrt(dx*dx + dy*dy);
	for (int i = 0; i < NB_SENSORS; ++i) {
		double s = wb_distance_sensor_get_value(ps[i]) / MAX_SENS; // 0..1
		if (has_target && dist < 0.20 && (i == 3 || i == 4)) s = 0.0;
		*vL += AVOID_GAIN * L_WEIGHT[i] * s;
		*vR += AVOID_GAIN * R_WEIGHT[i] * s;
	}
}

void go_to_patrol(double* vL, double* vR, const double* rgbs)
{
	if (has_target) {
		const double *p = wb_gps_get_values(gps);                // [x, y, z]
		const double *r = wb_inertial_unit_get_roll_pitch_yaw(imu);
		double x = p[0], y = p[1];                               // <-- CHANGEMENT: Y (pas Z)
		double heading = r[2];                                       // cap (rotation autour de l'axe vertical du monde)

		double dx = tx - x, dy = ty - y;
		double dist = sqrt(dx*dx + dy*dy);
		double desired = atan2(dy, dx);                          // angle dans le plan X–Y
		double err = desired - heading;
		err = fmod(err + M_PI, 2.0 * M_PI);
		if (err < 0)
			err += 2.0 * M_PI;
		err -= M_PI;

		double omega = K_TURN * err;
		double fwd   = K_FWD * clamp(dist, 0.0, FWD_CAP);

		*vL += fwd - omega;
		*vR += fwd + omega;

		if (color_arrived(rgbs)) {
		//if (dist < 0.1){
			char msg[64];
			//format of robot to supervisor messages: <r_id> <p_id>
			end_time = wb_robot_get_time();
			travel_time = end_time - start_time;
			snprintf(msg, sizeof(msg), "%d %d %d %lf", my_id, target_patrol, last_patrol, travel_time);
			wb_emitter_send(emitter, msg, strlen(msg) + 1);
			if (VERBOSE_BOTS){
				printf("[R%d] %s\n", my_id, msg);
			}
			has_target = false;
			*vL = *vR = 0.0;
			last_patrol = target_patrol;
		}
	}
}

int main() {
	double* rgbs= initialize();
	while ((wb_robot_step(TIME_STEP) != -1) && in_progress) {
		// Messages entrants
		receive_patrol(); //assign a target if supervisor sent one
		
		double vL = 0, vR = 0;
		if (has_target){
			vL= BASE_SPEED;
			vR= BASE_SPEED;

			// Évitement Braitenberg
			braitenberg_dodging(&vL, &vR);

			// Navigation vers (tx,ty) dans le plan X–Y
			go_to_patrol(&vL, &vR, rgbs);
		}

		set_speed(vL, vR);
	}

	free(rgbs);
	rgbs= NULL;
	wb_robot_cleanup();
	return 0;
}


