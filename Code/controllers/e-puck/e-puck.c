// controllers/e-puck/e-puck.c
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/camera.h>
// COULEUR

#include <math.h>
#include <stdio.h>
#include <string.h>

#define TIME_STEP   64
#define NB_SENSORS  8
#define MAX_V   	6.28
#define MAX_SENS	4095.0
// normaliser les IR
#define ARRIVE_EPS  0.08
// tolerance pour l'arrivee. a enlever apres qd on detectera avec les couleurs

static WbDeviceTag left_motor, right_motor;
static WbDeviceTag ps[NB_SENSORS];
static WbDeviceTag gps, imu;
static WbDeviceTag emitter, receiver;
static WbDeviceTag cam;   // COULEUR

static int my_id = -1;
static int target_patrol = -1;
static int has_target = 0;
static double tx = 0.0, ty = 0.0; // CIBLE dans le plan X–Y

// Color detection
static int cam_w, cam_h;
static int color_hits = 0;

// Braitenberg
static const double BASE_SPEED = 5.0;
static const double AVOID_GAIN = 200.0;
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

static void parse_go(const char *msg) {
  // "GO <id> <x> <y> <patrol_id>"
  int id, pid; double x, y;
  if (sscanf(msg, "GO %d %lf %lf %d", &id, &x, &y, &pid) == 4) {
	if (id == my_id) {
  	tx = x; ty = y; target_patrol = pid; has_target = 1;
  	color_hits = 0;  // COULEUR
  	printf("[R%d] Target set: (%.3f, %.3f), patrol=%d\n", my_id, tx, ty, target_patrol);
	}
  }
}



// COULEUR
typedef struct {
  int id;
  double x, y;
  double r, g, b;  // 0..1
} Patrol;
static Patrol patrols[64];
static int n_patrols = 0;

// --- parse "PP_TABLE" et "PP ..." reçus du supervisor
static void parse_message(const char *msg) {
  if (sscanf(msg, "PP_TABLE %d", &n_patrols) == 1) return;
  int id; double x,y,r,g,b;
  if (sscanf(msg, "PP %d %lf %lf %lf %lf %lf", &id,&x,&y,&r,&g,&b) == 6) {
	patrols[id].id=id; patrols[id].x=x; patrols[id].y=y;
	patrols[id].r=r; patrols[id].g=g; patrols[id].b=b;
  }
}

static void camera_init() {
  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, TIME_STEP);
  cam_w = wb_camera_get_width(cam);
  cam_h = wb_camera_get_height(cam);
}

// ----- utils couleur -----
static void rgb_to_hsv(double r, double g, double b, double *h, double *s, double *v) {
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
static void roi_hsv(double *H, double *S, double *V) {
  const unsigned char *img = wb_camera_get_image(cam);
  int cx = cam_w/2, cy = (3*cam_h)/4, half = 7; // fenetre a observer
  double Hsum=0, Ssum=0, Vsum=0; int count=1;  // faux de 1 mais balec
  const double RGB_SUM_MIN = 0.02; // ~ 0.02 par canal (sur [0..1])
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
      	  Hsum += h; Ssum += s; Vsum += v; count++;//}
	}
  *H = Hsum / count; *S = Ssum / count; *V = Vsum / count;
}

// distance de teinte en tenant compte du wrap 0/360
static double hue_dist(double h1, double h2){
  double d = fabs(h1 - h2);
  if (d > 180.0) d = 360.0 - d;
  return d;
}


static int color_arrived() {
  if (target_patrol < 0 || n_patrols <= target_patrol) return 0;

  // 1) lire la couleur cible en HSV
  double hr, sr, vr; // hsv cible
  {
	double r = patrols[target_patrol].r;
	double g = patrols[target_patrol].g;
	double b = patrols[target_patrol].b;
	rgb_to_hsv(r,g,b,&hr,&sr,&vr);
  }

  // 2) lire HSV moyen dans la ROI
  double h,s,v; roi_hsv(&h,&s,&v);

  // 3) règles robustes (H en degrés, S,V en [0..1])
  int ok = 0;

  // 3.a) garde-fous globaux (on ne décide pas dans le quasi-noir ou quasi-gris sombre)
  if (0) {                     // trop sombre -> H et S peu fiables, v < 0.12
    ok = 0;
  } else if (0) {  // très peu saturé ET pas lumineux -> éviter faux positifs, s < 0.08 && v < 0.60
    ok = 0;
  } else {
    // 3.b) détection "blanc" (S faible, V haut) indépendante du H
    //     Détecte blanc si la cible est blanche OU si la cible a S très faible.
    int target_is_white = (sr < 0.20 && vr > 0.85);
    if (target_is_white) {
      if (s < 0.20 && (h<10 || h>300)) ok = 1;  // && v > 0.70 
    }
    // 3.c) teintes spécifiques si la cible n'est pas "blanc"
    if (!ok && !target_is_white) {

      // bornes de tolérance (en degrés) – ajuste ±2..3° si besoin
      const double TOL_RED   = 20.0;
      const double TOL_YELL  = 10.0;
      const double TOL_CYAN  = 16.0;
      const double TOL_GREEN = 20.0;
      const double TOL_GEN   = 18.0;

      // repères de teinte
      const double H_RED0  = 0.0;    // 0/360
      const double H_YELL  = 60.0;
      const double H_GREEN = 120.0;
      const double H_CYAN  = 216.0;  // 200–210 selon rendu

      // --- JAUNE (1,1,0) ---
      if (hue_dist(hr, H_YELL) < TOL_YELL) {
        if (hue_dist(h, H_YELL) < (TOL_YELL)) //  && s > 0.55 && v > 0.35
          ok = 1;
      }
      // --- ROUGE (1,0,0) ---
      else if (hue_dist(hr, H_RED0) < TOL_RED) {  // || hue_dist(hr, 360.0) < TOL_RED
        // printf("[R%d] h=%f, s=%f, v=%f\n", my_id, h, s, v);
        if ((hue_dist(h, H_RED0) < (TOL_RED))) // || hue_dist(h, 360.0) < (TOL_RED+2.0))&& s > 0.35 && v > 0.28
           ok = 1;
      }
      // --- BLEU (0,0.333,1) ---
      else if (hue_dist(hr, H_CYAN) < (TOL_CYAN)) {
        if (hue_dist(h, H_CYAN) < (TOL_CYAN))  //  && s > 0.45 && v > 0.50
          ok = 1;
      }
      // --- VERT (0,0.333,0) ---
      // cible typique : hr≈120°, sr élevé, vr bas (~0.3). On impose une fenêtre de V basse à moyenne.
      else if (hue_dist(hr, H_GREEN) < TOL_GREEN) {
        if (hue_dist(h, H_GREEN) < (TOL_GREEN))  //  && s > 0.40 && v > 0.15 && v < 0.55
          ok = 1;
      }
      // --- CAS GÉNÉRIQUE (autres couleurs) ---
      if (!ok) {
        // Adaptation douce aux caractéristiques de la cible :
        // - saturation requise : au moins 0.28, ou 90% de la saturation cible (si cible peu saturée)
        double s_min = fmin(0.35, 0.90 * sr + 0.05);
        if (s_min < 0.28) s_min = 0.28;

        // - luminosité requise : éviter l'ombre, mais rester compatible avec cibles peu lumineuses
        double v_min = fmin(0.55, 0.90 * vr + 0.10);
        if (v_min < 0.20) v_min = 0.20;

        if (hue_dist(h, hr) < TOL_GEN && s > s_min && v > v_min)
          ok = 1;
      }
    }
  }

  // 4) stabilité temporelle
  if (ok) {
	color_hits++;
	return (color_hits >= 8); // 8 cycles stables ~0.5 s si TIME_STEP=64
  } else {
	color_hits = 0;
	return 0;
  }
}


// COULEUR



int main() {
  wb_robot_init();
  camera_init(); // COULEUR  
    
	// --- ID unique depuis le champ "name", p.ex. "EPUCK3" -> 3
  int parse_id_from_name(const char *nm) {
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

  while (wb_robot_step(TIME_STEP) != -1) {
	// Messages entrants
	// COULEUR
	while (wb_receiver_get_queue_length(receiver) > 0) {
  	const char *pkt = wb_receiver_get_data(receiver);
  	// découpe le paquet en lignes
  	const char *cur = pkt;
  	while (*cur) {
    	const char *nl = strchr(cur, '\n');
    	size_t len = nl ? (size_t)(nl - cur) : strlen(cur);
    	if (len > 0) {
      	char line[256];
      	size_t copy = len < sizeof(line)-1 ? len : sizeof(line)-1;
      	memcpy(line, cur, copy);
      	line[copy] = '\0';
      	if (strncmp(line, "GO ", 3) == 0)      	parse_go(line);
      	else if (strncmp(line, "PP_TABLE", 8) == 0) parse_message(line);
      	else if (strncmp(line, "PP ", 3) == 0)  	parse_message(line);
      	// ... d'autres types au besoin
    	}
    	if (!nl) break;
    	cur = nl + 1;
  	}
  	wb_receiver_next_packet(receiver);
	}
    
	// 1) Lecture pose
	const double *gp = wb_gps_get_values(gps);             	// [x, y, z]
	const double *iu = wb_inertial_unit_get_roll_pitch_yaw(imu);
	double x = gp[0], y = gp[1];
	double yaw = iu[2];
 
	// 2) Géométrie vers la cible (déclare TOUT ici, existe même si pas de cible)
	double dx = 0.0, dy = 0.0, dist = 1e9, desired = 0.0, err = 0.0;
	double omega = 0.0, fwd = 0.0;
	if (has_target) {
  	dx = tx - x; dy = ty - y;
  	dist = sqrt(dx*dx + dy*dy);
  	desired = atan2(dy, dx);
  	err = desired - yaw;
  	while (err >  M_PI) err -= 2.0 * M_PI;
  	while (err < -M_PI) err += 2.0 * M_PI;
  	omega = K_TURN * err;
  	fwd   = K_FWD * clamp(dist, 0.0, FWD_CAP);
	}
    
	// 3) Échelle d’évitement (portée globale à la boucle)
	double avoid_scale = 1.0;
	if (has_target) {
  	if (dist < 0.30) avoid_scale = 0.50;  // adoucir pres du patrol pour detecter la couleur
  	if (dist < 0.20) avoid_scale = 0.25;  // encore un peu d’IR pour les côtés
	}
 
	// 4) Vitesses de base
	double vL = BASE_SPEED, vR = BASE_SPEED;
    
	// 5) Braitenberg (peut utiliser dist, mais toujours défini)
	for (int i = 0; i < NB_SENSORS; ++i) {
  	double s = wb_distance_sensor_get_value(ps[i]) / MAX_SENS; // 0..1
  	// couper les capteurs frontaux seulement en approche finale ET si on a une cible
  	if (has_target && dist < 0.20 && (i == 3 || i == 4))
    	s = 0.0;
  	vL += avoid_scale * AVOID_GAIN * L_WEIGHT[i] * s;
  	vR += avoid_scale * AVOID_GAIN * R_WEIGHT[i] * s;
	}
    
	// 6) Go-to-goal
	if (has_target) {
  	vL += fwd - omega;
  	vR += fwd + omega;
	}
    
	// 7) Validation par couleur (seulement si on a une cible)
	if (has_target && color_arrived()) {
  	char msg[64];
  	snprintf(msg, sizeof(msg), "ARRIVED %d %d", my_id, target_patrol);
  	wb_emitter_send(emitter, msg, strlen(msg) + 1);
  	printf("[R%d] %s\n", my_id, msg);
  	has_target = 0;
  	vL = vR = 0.0;
	}

	set_speed(vL, vR);
	// COULEUR
  }

  wb_robot_cleanup();
  return 0;
}




