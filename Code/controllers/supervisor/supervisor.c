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

// 1 COULEUR
//helper: get rbg value of a Solid DEF=<def>
static int get_rgb_of_def(const char *def, double *r, double *g, double *b) {
  WbNodeRef n = wb_supervisor_node_get_from_def(def);
  if (!n) return 0;
  // enfants -> Shape 0
  WbFieldRef children = wb_supervisor_node_get_field(n, "children");
  if (!children) return 0;
  int count = wb_supervisor_field_get_count(children);
  for (int i = 0; i < count; ++i) {
    WbNodeRef child = wb_supervisor_field_get_mf_node(children, i);
    if (!child) continue;
    const char *type = wb_supervisor_node_get_type_name(child);
    if (strcmp(type, "Shape") == 0) {
      WbFieldRef app_f = wb_supervisor_node_get_field(child, "appearance");
      if (!app_f) return 0;
      WbNodeRef app = wb_supervisor_field_get_sf_node(app_f);
      if (!app) return 0;
      WbFieldRef mat_f = wb_supervisor_node_get_field(app, "material");
      if (!mat_f) return 0;
      WbNodeRef mat = wb_supervisor_field_get_sf_node(mat_f);
      if (!mat) return 0;
      WbFieldRef diff_f = wb_supervisor_node_get_field(mat, "diffuseColor");
      if (!diff_f) return 0;
      const double *rgb = wb_supervisor_field_get_sf_color(diff_f);
      *r = rgb[0]; *g = rgb[1]; *b = rgb[2];
      return 1;
    }
  }
  return 0;
}
// 1 COULEUR

// 2 COULEUR
// message multi-ligne, ex :
// PP_TABLE N
// PP id x y r g b
// PP id x y r g b
static void broadcast_patrol_table(int n_patrols,
                                   char  def[][32],
                                   const double *px,
                                   const double *py) {
  char buf[4096]; buf[0] = 0;
  snprintf(buf, sizeof(buf), "PP_TABLE %d\n", n_patrols);

  for (int i = 0; i < n_patrols; ++i) {
    double r = 1, g = 1, b = 1;           // fallback if no color
    (void)get_rgb_of_def(def[i], &r, &g, &b);
    char line[128];
    snprintf(line, sizeof(line),
             "PP %d %.3f %.3f %.3f %.3f %.3f\n",
             i, px[i], py[i], r, g, b);
    strncat(buf, line, sizeof(buf) - strlen(buf) - 1);
  }

  wb_emitter_send(emitter, buf, strlen(buf) + 1);
  printf("[SUP] Broadcasted PP_TABLE with %d entries\n", n_patrols);
}

// 2 COULEUR


int main() {
  wb_robot_init();
  emitter  = wb_robot_get_device("emitter");
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  // --- Discover robots and patrol points by DEF naming convention
  int n_robots = 0, n_patrols = 0;
  char name[32];
  double patrol_x[MAX_PATROLS], patrol_y[MAX_PATROLS];
  char   patrol_def[MAX_PATROLS][32]; // COULEUR

  // Patrols: PATROL0..PATROL*
  for (int i = 0; i < MAX_PATROLS; ++i) {
    snprintf(name, sizeof(name), "PATROL%d", i);
    double x, y;
    if (!get_xy_of_def(name, &x, &y)) break;
    patrol_x[n_patrols] = x; patrol_y[n_patrols] = y; // pour quils puissent atteindre leur but
    strcpy(patrol_def[n_patrols], name); // COULEUR
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
  broadcast_patrol_table(n_patrols, patrol_def, patrol_x, patrol_y); // COULEUR


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


