// controllers/supervisor/supervisor.c
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <stdio.h>
#include <string.h>

#define TIME_STEP 32
#define MAX_ROBOTS 16
#define MAX_PATROLS 64

static WbDeviceTag emitter, receiver;

// Helper: get 2D position (x,y) of a DEF node in the X–Y plane (Z = height)
static int get_xy_of_def(const char *def, double *x, double *y) {
  WbNodeRef n = wb_supervisor_node_get_from_def(def);
  if (!n) return 0;
  // Read the 'translation' field explicitly: [x, y, z]
  WbFieldRef f = wb_supervisor_node_get_field(n, "translation");
  if (!f) return 0;
  const double *t = wb_supervisor_field_get_sf_vec3f(f); // [x, y, z]
  if (!t) return 0;
  *x = t[0];       // X (plane)
  *y = t[1];       // Y (plane)
  return 1;
}

int main() {
  wb_robot_init();
  emitter  = wb_robot_get_device("emitter");
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  // --- Discover robots and patrol points by DEF naming convention
  int n_robots = 0, n_patrols = 0;
  char name[32];
  double patrol_x[MAX_PATROLS], patrol_y[MAX_PATROLS];

  // Patrols: PATROL0..PATROL*
  for (int i = 0; i < MAX_PATROLS; ++i) {
    snprintf(name, sizeof(name), "PATROL%d", i);
    double x, y;
    if (!get_xy_of_def(name, &x, &y)) break;
      patrol_x[n_patrols] = x; patrol_y[n_patrols] = y; // pour quils puissent atteindre leur but
    n_patrols++;
  }

  // Robots: EPUCK0..EPUCK*
  double rx, ry;
  for (int i = 0; i < MAX_ROBOTS; ++i) {
    snprintf(name, sizeof(name), "EPUCK%d", i);
    if (!get_xy_of_def(name, &rx, &ry)) break;
    n_robots++;
  }

  printf("[SUP] Found %d robots and %d patrol points.\n", n_robots, n_patrols);

  // --- Assign one patrol arbitrarily to each robot (round-robin)
  for (int r = 0; r < n_robots; ++r) {
    int p = (n_patrols > 0) ? (r % n_patrols) : -1;
    if (p >= 0) {
      char msg[128];
      // GO <robot_id> <x> <y> <patrol_id>
      snprintf(msg, sizeof(msg), "GO %d %.3f %.3f %d", r, patrol_x[p], patrol_y[p], p);
      wb_emitter_send(emitter, msg, strlen(msg) + 1);
      printf("[SUP] Sent: %s\n", msg);
    }
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    // ... dans la boucle while (wb_robot_step(TIME_STEP) != -1) ...
  while (wb_receiver_get_queue_length(receiver) > 0) {
    const char *data = wb_receiver_get_data(receiver);
    int rid = -1, pid = -1;
    if (sscanf(data, "ARRIVED %d %d", &rid, &pid) == 2) {
      printf("[SUP] Robot %d arrived at patrol %d\n", rid, pid);
      if (rid >= 0 && rid < n_robots && n_patrols > 0) {
        // Assigner le patrol point suivant (boucle circulaire)
        int next = (pid + 1) % n_patrols;
        char msg[128];
        // GO <robot_id> <x> <y> <patrol_id>
        snprintf(msg, sizeof(msg), "GO %d %.3f %.3f %d",
                 rid, patrol_x[next], patrol_y[next], next);
        wb_emitter_send(emitter, msg, strlen(msg) + 1);
        printf("[SUP] Resent: %s\n", msg);
      }
    }
    wb_receiver_next_packet(receiver);
  }
  

  }

  wb_robot_cleanup();
  return 0;
}


