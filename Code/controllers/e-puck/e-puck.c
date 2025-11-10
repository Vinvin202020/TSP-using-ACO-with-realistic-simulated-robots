// controllers/e-puck/e-puck.c
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

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

static int my_id = -1;
static int target_patrol = -1;
static int has_target = 0;
static double tx = 0.0, ty = 0.0; // CIBLE dans le plan X–Y

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
      tx = x; ty = y; target_patrol = pid; has_target = 1;
	  if (VERBOSE_BOTS){
      	printf("[R%d] Target set: (%.3f, %.3f), patrol=%d\n", my_id, tx, ty, target_patrol);
	  }
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

void initialize()
{
	wb_robot_init();
  
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
	for (int i = 0; i < NB_SENSORS; ++i) {
		double s = wb_distance_sensor_get_value(ps[i]) / MAX_SENS; // 0..1
		*vL += AVOID_GAIN * L_WEIGHT[i] * s;
		*vR += AVOID_GAIN * R_WEIGHT[i] * s;
	}
}

void go_to_patrol(double* vL, double* vR)
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

		if (dist < ARRIVE_EPS) {
			char msg[64];
			//format of robot to supervisor messages: <r_id> <p_id>
			snprintf(msg, sizeof(msg), "%d %d", my_id, target_patrol);
			wb_emitter_send(emitter, msg, strlen(msg) + 1);
			if (VERBOSE_BOTS){
				printf("[R%d] %s\n", my_id, msg);
			}
			has_target = 0;
			*vL = *vR = 0.0;
		}
	}
}

int main() {
	initialize();
	while (wb_robot_step(TIME_STEP) != -1) {
		// Messages entrants
		receive_patrol(); //assign a target if supervisor sent one
		
		double vL = 0, vR = 0;
		if (has_target){
			vL= BASE_SPEED;
			vR= BASE_SPEED;

			// Évitement Braitenberg
			braitenberg_dodging(&vL, &vR);

			// Navigation vers (tx,ty) dans le plan X–Y
			go_to_patrol(&vL, &vR);
		}

		set_speed(vL, vR);
	}

	wb_robot_cleanup();
	return 0;
}


