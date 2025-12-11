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
#include <time.h>

#define TIME_STEP    32
#define NB_SENSORS   8
#define MAX_V        6.28
#define MAX_SENS     4095.0
#define VERBOSE_BOTS 0

// ----- TSP / ACO PARAMETERS -----
#define N_PATROLS    5           // number of patrol points
#define N_ITERS      3           // number of ACO iterations
#define BIG_LENGTH   10000.0

#define Q0           0.9
#define BETA         1.0        // matches your supervisor: tau * (d_ij)^BETA
#define ALPHA        0.1        // pheromone update factor
#define XI           0.1        // local pheromone update parameter ξ

#define PATH_LEN     (N_PATROLS + 1)   // cycle: N_PATROLS edges, N_PATROLS+1 nodes (return to start)

// ----- COMMUNICATION -----
#define COMM_RANGE   0.7         // meters, adjust and watch prints
#define MSG_BUF_LEN  1024

// Devices
static WbDeviceTag left_motor, right_motor;
static WbDeviceTag ps[NB_SENSORS];
static WbDeviceTag gps, imu;
static WbDeviceTag emitter, receiver;
static WbDeviceTag cam;

// Robot info
static int my_id         = -1;
static int start_node    = 0;    // starting patrol for this robot
static int target_patrol = -1;
static int last_patrol   = -1;
static bool has_target   = false;
static double tx = 0.0, ty = 0.0;
static double start_time = 0.0;

// For local-update timing
static bool just_arrived_edge = false;
static int edge_from = -1;
static int edge_to   = -1;

// Braitenberg
static const double BASE_SPEED = 5.0;
static const double AVOID_GAIN = 1000.0;
static const double L_WEIGHT[NB_SENSORS] = { -1.0, -0.8, -0.5,  0.0,  0.0,  0.5,  0.8,  1.0 };
static const double R_WEIGHT[NB_SENSORS] = {  1.0,  0.8,  0.5,  0.0,  0.0, -0.5, -0.8, -1.0 };

// Go-to-goal
static const double K_TURN  = 4.0;
static const double K_FWD   = 2.0;
static const double FWD_CAP = 0.3;

// ----- COLOR DETECTION (ADDED) -----

// Camera properties
static int cam_w = 0, cam_h = 0;

// Color detection state
static int color_hits = 0;
static const double TOL_GEN = 10.0;  // hue tolerance in degrees
static int consistency = 4;          // number of stable detections required

// Hardcoded RGB values of patrol points (0..1)
static const double PATROL_RGB[N_PATROLS][3] = {
  {1.000, 0.000, 0.000}, // node 0: red
  {0.000, 0.333, 1.000}, // node 1: blue
  {0.000, 0.333, 0.000}, // node 2: green
  {1.000, 1.000, 1.000}, // node 3: white
  {1.000, 1.000, 0.000}  // node 4: yellow
};

static double clamp(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static void set_speed(double vl, double vr) {
  wb_motor_set_velocity(left_motor,  clamp(vl, -MAX_V, MAX_V));
  wb_motor_set_velocity(right_motor, clamp(vr, -MAX_V, MAX_V));
}

// --- ID unique depuis le champ "name", p.ex. "EPUCK3" -> 3
static int parse_id_from_name(const char *nm) {
  int id = -1, tmp = -1;
  for (const char *p = nm; *p; ++p) {
    if (*p >= '0' && *p <= '9') {
      if (tmp < 0) tmp = 0;
      tmp = tmp * 10 + (*p - '0');
    } else if (tmp >= 0) {
      id = tmp;
      tmp = -1;
    }
  }
  if (tmp >= 0) id = tmp;
  return id;
}

// ---------- HARD-CODED MAP / INVERTED DISTANCES ----------
// Positions (x, y) for nodes 0..4:
static const double PATROL_X[N_PATROLS] = {
  0.000000, 1.600000, -1.700000, -1.500000, 1.300000
};
static const double PATROL_Y[N_PATROLS] = {
  0.000000, 0.750000, 0.000000, 1.600000, -1.800000
};

// This is the "inverted distance" table from your supervisor (d_ij = 1 / time_ij):
static const double D_INV[N_PATROLS][N_PATROLS] = {
  {0.0000, 0.0572, 0.0552, 0.0468, 0.0495},
  {0.0572, 0.0000, 0.0295, 0.0328, 0.0410},
  {0.0552, 0.0295, 0.0000, 0.0693, 0.0300},
  {0.0468, 0.0328, 0.0693, 0.0000, 0.0230},
  {0.0495, 0.0410, 0.0300, 0.0230, 0.0000}
};

// Local pheromone table
static double pher_table[N_PATROLS][N_PATROLS];
static double tau0_global = 0.0;  // initial pheromone for local update

// Tours
typedef struct {
  int path[PATH_LEN];   // cycle
  int num_patrols;      // number of nodes in path
  double length;        // "time" = sum of 1 / d_ij
} Tour;

static Tour tour_curr;

// ACO iteration state
static int  current_iter = 0;         // 0..N_ITERS-1
static bool visited[N_PATROLS];
static int  visited_count = 0;
static int  current_node  = 0;

// Per-iteration best info (distributed)
static double best_len[N_ITERS];
static int    best_path[N_ITERS][PATH_LEN];

// ---------- TSP / ACO UTILITIES ----------

// Use same cost as supervisor: sum of 1 / d_ij along the path
static double tour_length(const Tour *t) {
  double L = 0.0;
  for (int i = 0; i < t->num_patrols - 1; ++i) {
    int a = t->path[i];
    int b = t->path[i + 1];
    if (D_INV[a][b] > 0.0)
      L += 1.0 / D_INV[a][b];
  }
  return L;
}

// Compute L_NN like in your supervisor (starting at node 0, using inverted distances)
static double compute_L_NN(void) {
  bool J_access[N_PATROLS];
  for (int i = 0; i < N_PATROLS; ++i)
    J_access[i] = true;

  int current = 0;
  J_access[0] = false;

  double l_nn = 0.0;

  for (int step = 0; step < N_PATROLS - 1; ++step) {
    double max_inv = 0.0;
    int next = -1;
    for (int j = 0; j < N_PATROLS; ++j) {
      if (J_access[j] && D_INV[current][j] > max_inv) {
        max_inv = D_INV[current][j];
        next = j;
      }
    }
    if (next < 0 || max_inv <= 0.0)
      break;
    l_nn += 1.0 / max_inv;
    current = next;
    J_access[current] = false;
  }

  if (D_INV[current][0] > 0.0)
    l_nn += 1.0 / D_INV[current][0];

  return l_nn;
}

static void init_pheromone_table(void) {
  double L_NN = compute_L_NN();
  double tau0 = 1.0 / (N_PATROLS * L_NN);
  tau0_global = tau0;

  for (int i = 0; i < N_PATROLS; ++i) {
    for (int j = 0; j < N_PATROLS; ++j) {
      if (i == j)
        pher_table[i][j] = 0.0;
      else
        pher_table[i][j] = tau0;
    }
  }
  printf("[R?] L_NN = %.4f, tau0 = %.6f\n", L_NN, tau0);
}

// local pheromone update on single edge (i, j)
static void local_pheromone_update(int from, int to) {
  if (from < 0 || from >= N_PATROLS || to < 0 || to >= N_PATROLS || from == to)
    return;

  double *tab_ab = &pher_table[from][to];
  double *tab_ba = &pher_table[to][from];
  *tab_ab = (1.0 - XI) * (*tab_ab) + XI * tau0_global;  // τ_ij ← (1-ξ)τ_ij + ξτ0
  *tab_ba = *tab_ab;                                    // keep symmetric
}

// Only called at the end of an iteration, based on best tour of the PREVIOUS iteration
static void apply_pheromone_from_tour(const int *path, double length) {

  double delta = 1.0 / length;
  for (int i = 0; i < PATH_LEN - 1; ++i) {
    int a = path[i];
    int b = path[i + 1];
    if (a < 0 || a >= N_PATROLS || b < 0 || b >= N_PATROLS)
      continue;
    double *tab_ab = &pher_table[a][b];
    double *tab_ba = &pher_table[b][a];
    *tab_ab = (1.0 - ALPHA) * (*tab_ab) + ALPHA * delta;
    *tab_ba = *tab_ab;
  }
}

static void reset_visited_and_tour(void) {
  for (int i = 0; i < N_PATROLS; ++i)
    visited[i] = false;

  visited_count = 1;
  current_node  = start_node;
  visited[start_node] = true;

  tour_curr.num_patrols = 1;
  tour_curr.path[0] = start_node;
  tour_curr.length = 0.0;

  last_patrol  = start_node;
  has_target   = false;
  target_patrol = -1;
}

static int choose_next_patrol(void) {
  // If all patrols visited, go back to start_node to close cycle
  if (visited_count >= N_PATROLS)
    return start_node;

  double q = (double)rand() / (double)RAND_MAX;

  // compute desirability denominator using D_INV (like supervisor)
  double denom = 0.0;
  for (int j = 0; j < N_PATROLS; ++j) {
    if (!visited[j] && D_INV[current_node][j] > 0.0) {
      double tau = pher_table[current_node][j];
      double eta = D_INV[current_node][j]; // supervisor: tau * (d_ij)^BETA with d_ij = inverted distance
      denom += tau * pow(eta, BETA);
    }
  }

  if (denom <= 0.0) {
    // fallback: pick first unvisited
    for (int j = 0; j < N_PATROLS; ++j)
      if (!visited[j])
        return j;
    return start_node;
  }

  if (q <= Q0) {
    // exploitation: argmax
    int best_j = start_node;
    double best_val = -1.0;
    for (int j = 0; j < N_PATROLS; ++j) {
      if (!visited[j] && D_INV[current_node][j] > 0.0) {
        double tau = pher_table[current_node][j];
        double eta = D_INV[current_node][j];
        double val = tau * pow(eta, BETA);
        if (val > best_val) {
          best_val = val;
          best_j = j;
        }
      }
    }
    return best_j;
  } else {
    // exploration: roulette wheel
    double r = (double)rand() / (double)RAND_MAX;
    double cum = 0.0;
    int last_valid = start_node;

    for (int j = 0; j < N_PATROLS; ++j) {
      if (!visited[j] && D_INV[current_node][j] > 0.0) {
        double tau = pher_table[current_node][j];
        double eta = D_INV[current_node][j];
        double p = (tau * pow(eta, BETA)) / denom;
        cum += p;
        last_valid = j;
        if (cum >= r)
          return j;
      }
    }
    return last_valid;
  }
}

// ---------- COMMUNICATION (RANGE-CONSTRAINED) ----------

static void broadcast_best_info(void) {
  const double *p = wb_gps_get_values(gps); // [x, y, z]
  double x = p[0];
  double y = p[1];

  char buf[MSG_BUF_LEN];
  int offset = snprintf(buf, sizeof(buf), "R %d %d %.3f %.3f %d",
                        my_id, current_iter, x, y, N_ITERS);
  if (offset < 0 || offset >= MSG_BUF_LEN)
    return;

  for (int it = 0; it < N_ITERS && offset < MSG_BUF_LEN - 10; ++it) {
    offset += snprintf(buf + offset, MSG_BUF_LEN - offset, " %.6f", best_len[it]);
    if (offset >= MSG_BUF_LEN - 10) break;

    for (int k = 0; k < PATH_LEN && offset < MSG_BUF_LEN - 5; ++k) {
      offset += snprintf(buf + offset, MSG_BUF_LEN - offset, " %d", best_path[it][k]);
      if (offset >= MSG_BUF_LEN - 5) break;
    }
  }

  wb_emitter_send(emitter, buf, strlen(buf) + 1);
}

// parse one double and PATH_LEN ints from msg pointer p, advance p via *consumed
static bool parse_one_iter(const char *p, double *len_out, int *path_out, int *consumed) {
  int used = 0;
  int n = 0;
  double L;
  if (sscanf(p, " %lf%n", &L, &n) != 1)
    return false;
  used += n;
  p += n;

  for (int i = 0; i < PATH_LEN; ++i) {
    int v;
    if (sscanf(p, " %d%n", &v, &n) != 1)
      return false;
    path_out[i] = v;
    used += n;
    p += n;
  }

  *len_out = L;
  *consumed = used;
  return true;
}

static void handle_incoming_comm(void) {
  const double *mypos = wb_gps_get_values(gps);
  double myx = mypos[0];
  double myy = mypos[1];

  while (wb_receiver_get_queue_length(receiver) > 0) {
    const char *msg = wb_receiver_get_data(receiver);

    if (msg[0] == 'R') {
      int sender, sender_iter, nIters;
      double sx, sy;
      int consumed = 0;
      int n = sscanf(msg, "R %d %d %lf %lf %d%n",
                     &sender, &sender_iter, &sx, &sy, &nIters, &consumed);
      if (n == 5 && sender != my_id) {
        double dx = sx - myx;
        double dy = sy - myy;
        double dist = sqrt(dx * dx + dy * dy);

        if (dist <= COMM_RANGE) {
          const char *p = msg + consumed;
          int iters_to_read = (nIters < N_ITERS) ? nIters : N_ITERS;
          int start_index = (current_iter > 0) ? (current_iter - 1) : 0;

          // Only update array entries; pheromone update happens at iteration end
          for (int it = 0; it < iters_to_read; ++it) {
            double L;
            int tmp_path[PATH_LEN];
            int used = 0;
            if (!parse_one_iter(p, &L, tmp_path, &used))
              break;
            p += used;

            if (it < start_index)
              continue;  // ignore iterations older than n-1

            if (L < best_len[it]) {
              best_len[it] = L;
              for (int k = 0; k < PATH_LEN; ++k)
                best_path[it][k] = tmp_path[k];

              printf("[R%d]   -> Adopted better tour for iter %d from R%d (L=%.4f)\n",
                     my_id, it, sender, L);
            }
          }
        }
      }
    }

    wb_receiver_next_packet(receiver);
  }
}

// ---------- COLOR / CAMERA HELPERS (ADDED) ----------

static void rgb_to_hsv(double r, double g, double b, double *h, double *s, double *v) {
  double max = fmax(r, fmax(g, b)), min = fmin(r, fmin(g, b));
  double d = max - min;
  *v = max;
  *s = (max <= 1e-6) ? 0.0 : d / max;
  double hh;
  if (d < 1e-6) hh = 0.0;
  else if (max == r) hh = fmod(((g - b) / d), 6.0);
  else if (max == g) hh = ((b - r) / d) + 2.0;
  else               hh = ((r - g) / d) + 4.0;
  hh *= 60.0;
  if (hh < 0.0) hh += 360.0;
  *h = hh;
}

// compute H,S,V average on a ROI in the lower center of the image
static void roi_hsv(double *H, double *S, double *V) {
  const unsigned char *img = wb_camera_get_image(cam);
  if (!img || cam_w <= 0 || cam_h <= 0) {
    *H = *S = *V = 0.0;
    return;
  }

  int cx = cam_w / 2;
  int cy = (3 * cam_h) / 4;
  int half = 3;

  double Hsum = 0.0, Ssum = 0.0, Vsum = 0.0;
  int count = 1;  // avoid divide by zero

  const double V_BLACK = 0.08;

  for (int y = cy - half; y <= cy + half; ++y) {
    for (int x = cx - half; x <= cx + half; ++x) {
      double r = wb_camera_image_get_red(img,   cam_w, x, y) / 255.0;
      double g = wb_camera_image_get_green(img, cam_w, x, y) / 255.0;
      double b = wb_camera_image_get_blue(img,  cam_w, x, y) / 255.0;

      double h, s, v;
      rgb_to_hsv(r, g, b, &h, &s, &v);

      if (v < V_BLACK)
        continue;

      Hsum += h;
      Ssum += s;
      Vsum += v;
      count++;
    }
  }

  *H = Hsum / count;
  *S = Ssum / count;
  *V = Vsum / count;
}

static double hue_dist(double h1, double h2) {
  double d = fabs(h1 - h2);
  if (d > 180.0) d = 360.0 - d;
  return d;
}

static bool color_arrived(void) {
  if (target_patrol < 0 || target_patrol >= N_PATROLS)
    return false;

  // target color in HSV
  double hr, sr, vr;
  double r = PATROL_RGB[target_patrol][0];
  double g = PATROL_RGB[target_patrol][1];
  double b = PATROL_RGB[target_patrol][2];

  rgb_to_hsv(r, g, b, &hr, &sr, &vr);

  // measured color in ROI
  double h, s, v;
  roi_hsv(&h, &s, &v);

  if (hue_dist(h, hr) < TOL_GEN) {
    color_hits++;
    return (color_hits >= consistency);
  } else {
    color_hits = 0;
    return false;
  }
}

// ---------- MOTION / SENSORS ----------

static void braitenberg_dodging(double *vL, double *vR) {
  const double *p = wb_gps_get_values(gps);
  double x = p[0], y = p[1];
  double dx = tx - x, dy = ty - y;
  double dist = sqrt(dx * dx + dy * dy);

  for (int i = 0; i < NB_SENSORS; ++i) {
    double s = wb_distance_sensor_get_value(ps[i]) / MAX_SENS;
    if (has_target && dist < 0.20 && (i == 3 || i == 4))
      s = 0.0;
    *vL += AVOID_GAIN * L_WEIGHT[i] * s;
    *vR += AVOID_GAIN * R_WEIGHT[i] * s;
  }
}

static void go_to_patrol(double *vL, double *vR) {
  if (!has_target)
    return;

  const double *p = wb_gps_get_values(gps);
  const double *r = wb_inertial_unit_get_roll_pitch_yaw(imu);

  double x = p[0], y = p[1];
  double heading = r[2];

  double dx = tx - x, dy = ty - y;
  double dist = sqrt(dx * dx + dy * dy);

  double desired = atan2(dy, dx);
  double err = desired - heading;
  err = fmod(err + M_PI, 2.0 * M_PI);
  if (err < 0.0) err += 2.0 * M_PI;
  err -= M_PI;

  double omega = K_TURN * err;
  double fwd   = K_FWD * clamp(dist, 0.0, FWD_CAP);

  *vL += fwd - omega;
  *vR += fwd + omega;

  if (dist < 0.20) {
    if (dist < 0.15) {
      *vL = 2.0;
      *vR = -2.0;
    }
    if (color_arrived()) {
      has_target = false;
      last_patrol = target_patrol;
      just_arrived_edge = true;
      if (VERBOSE_BOTS) {
        printf("[R%d] Arrived at patrol %d (color-based)\n", my_id, target_patrol);
      }
    }
  }
}

// ---------- INITIALISATION ----------

static void camera_init(void) {
  cam = wb_robot_get_device("camera");
  if (cam) {
    wb_camera_enable(cam, TIME_STEP);
    cam_w = wb_camera_get_width(cam);
    cam_h = wb_camera_get_height(cam);
  }
}

static void initialize(void) {
  wb_robot_init();

  const char *nm = wb_robot_get_name();
  my_id = parse_id_from_name(nm);
  if (my_id < 0) {
    fprintf(stderr, "[%s] ERROR: robot name must end with an integer, e.g. EPUCK0\n", nm);
    my_id = 0;
  }

  // decorrelate randomness across robots
  srand((unsigned int)time(NULL) + my_id * 12345);

  // Motors
  left_motor  = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // IR sensors
  char name[8] = "ps0";
  for (int i = 0; i < NB_SENSORS; ++i) {
    ps[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
    name[2]++;
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

  // Camera
  camera_init();

  // ACO init
  init_pheromone_table();

  for (int it = 0; it < N_ITERS; ++it) {
    best_len[it] = BIG_LENGTH;
    for (int k = 0; k < PATH_LEN; ++k)
      best_path[it][k] = 0;
  }

  // Different start node per robot
  start_node = my_id % N_PATROLS;

  current_iter = 0;
  reset_visited_and_tour();
}

// ---------- MAIN LOOP ----------

int main(void) {
  initialize();

  while (wb_robot_step(TIME_STEP) != -1) {
    // Range-limited information exchange of per-iteration best tours
    broadcast_best_info();
    handle_incoming_comm();

    double vL = 0.0, vR = 0.0;

    if (has_target) {
      vL = BASE_SPEED;
      vR = BASE_SPEED;
      braitenberg_dodging(&vL, &vR);
      go_to_patrol(&vL, &vR);
    }

    // local pheromone update just after finishing traversal of an edge
    if (just_arrived_edge) {
      local_pheromone_update(edge_from, edge_to);
      just_arrived_edge = false;
    }

    if (!has_target) {
      // "Complete tour" = visited all nodes and returned to start_node,
      // so we have PATH_LEN nodes in the cycle.
      bool tour_closed =
        (current_node == start_node &&
         tour_curr.num_patrols == PATH_LEN);

      if (tour_closed) {
        // finalize current tour
        tour_curr.length = tour_length(&tour_curr);
        printf("[R%d] Finished tour %d: L = %.4f\n",
               my_id, current_iter, tour_curr.length);

        // Store/improve best for THIS iteration (array only)
        if (tour_curr.length < best_len[current_iter]) {
          best_len[current_iter] = tour_curr.length;
          for (int k = 0; k < PATH_LEN; ++k)
            best_path[current_iter][k] = tour_curr.path[k];

          printf("[R%d]   -> New local best for iter %d (L=%.4f)\n",
                 my_id, current_iter, tour_curr.length);
        }

        // Update pheromones from best tour of PREVIOUS iteration
        if (current_iter > 0 && best_len[current_iter - 1] < BIG_LENGTH) {
          apply_pheromone_from_tour(best_path[current_iter - 1],
                                    best_len[current_iter - 1]);
          printf("[R%d]   -> Applied pheromone from best iter %d (L=%.4f)\n",
                 my_id, current_iter - 1, best_len[current_iter - 1]);
        }

        current_iter++;
        if (current_iter >= N_ITERS) {
          // done: stop moving
          set_speed(0.0, 0.0);
          break;
        }

        // prepare next iteration
        reset_visited_and_tour();
      }

      // still inside current iteration
      if (current_iter < N_ITERS) {
        if (visited_count < N_PATROLS) {
          int next = choose_next_patrol();
          visited[next] = true;
          visited_count++;
          tour_curr.path[tour_curr.num_patrols++] = next;
          current_node = next;

          edge_from = last_patrol;
          edge_to   = next;

          tx = PATROL_X[next];
          ty = PATROL_Y[next];
          target_patrol = next;
          has_target = true;
          start_time = wb_robot_get_time();

          if (VERBOSE_BOTS) {
            printf("[R%d] Iter %d: going to patrol %d\n",
                   my_id, current_iter, next);
          }
        } else if (visited_count == N_PATROLS && current_node != start_node) {
          int next = start_node; // return to start
          tour_curr.path[tour_curr.num_patrols++] = next;
          current_node = next;

          edge_from = last_patrol;
          edge_to   = next;

          tx = PATROL_X[next];
          ty = PATROL_Y[next];
          target_patrol = next;
          has_target = true;
          start_time = wb_robot_get_time();

          if (VERBOSE_BOTS) {
            printf("[R%d] Iter %d: returning to start\n",
                   my_id, current_iter);
          }
        }
      }
    }

    set_speed(vL, vR);
  }

  wb_robot_cleanup();
  return 0;
}
