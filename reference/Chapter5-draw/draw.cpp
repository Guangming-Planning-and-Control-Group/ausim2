#include <cmath>
#include <cstdio>
#include <cstring>

#include <GLFW/glfw3.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

#include <iostream>

// MuJoCo data structures
mjModel *m = NULL; // MuJoCo model
mjData *d = NULL;  // MuJoCo data
mjvCamera cam;     // abstract camera
mjvOption opt;     // visualization options
mjvScene scn;      // abstract scene
mjrContext con;    // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancdataode, int act,
              int mods) {
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods) {
  // update button state
  button_left =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

std::vector<float> get_sensor_data(const mjModel *model, const mjData *data,
                                   const std::string &sensor_name) {
  int sensor_id = mj_name2id(model, mjOBJ_SENSOR, sensor_name.c_str());
  if (sensor_id == -1) {
    std::cout << "no found sensor" << std::endl;
    return std::vector<float>();
  }
  int data_pos = model->sensor_adr[sensor_id];
  std::vector<float> sensor_data(model->sensor_dim[sensor_id]);
  for (int i = 0; i < sensor_data.size(); i++) {
    sensor_data[i] = data->sensordata[data_pos + i];
  }
  return sensor_data;
}

/*--------绘制直线--------*/
void draw_line(mjvScene *scn, mjtNum *from, mjtNum *to, mjtNum width,
               float *rgba) {
  scn->ngeom += 1;
  mjvGeom *geom = scn->geoms + scn->ngeom - 1;
  mjv_initGeom(geom, mjGEOM_SPHERE, NULL, NULL, NULL, rgba);
  mjv_connector(geom, mjGEOM_LINE, width, from, to);
}

/*--------绘制箭头--------*/
void draw_arrow(mjvScene *scn, mjtNum *from, mjtNum *to, mjtNum width,
                float rgba[4]) {
  scn->ngeom += 1;
  mjvGeom *geom = scn->geoms + scn->ngeom - 1;
  mjv_initGeom(geom, mjGEOM_SPHERE, NULL, NULL, NULL, rgba);
  mjv_connector(geom, mjGEOM_ARROW, width, from, to);
}

/*--------绘制几何体--------*/
void draw_geom(mjvScene *scn, int type, mjtNum *size, mjtNum *pos, mjtNum *mat,
               float rgba[4]) {
  scn->ngeom += 1;
  mjvGeom *geom = scn->geoms + scn->ngeom - 1;
  mjv_initGeom(geom, type, size, pos, mat, rgba);
}

// main function
int main(int argc, const char **argv) {
  char error[1000] = "Could not load binary model";
  m = mj_loadXML("../../../API-MJCF/mecanum.xml", 0, error, 1000);

  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow *window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  float cnt = 0;
  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window)) {

    d->ctrl[0] = std::sin(cnt);
    d->ctrl[1] = std::cos(cnt);
    d->ctrl[2] = std::sin(cnt);
    mj_step(m, d);
    cnt += 0.001;

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

    /*--------3D绘制--------*/
    mjtNum from[3] = {0, 1, 1};
    mjtNum to[3] = {0, 3, 4};
    float color[4] = {0, 1, 0, 1};
    draw_line(&scn, from, to, 20, color);
    mjtNum to2[3] = {0, -1, 1};
    float color2[4] = {0, 0, 1, 1};
    draw_arrow(&scn, from, to2, 0.1, color2);

    mjtNum size[3] = {0.1, 0, 0};
    mjtNum pos[3] = {0, 0, 1.0};
    mjtNum mat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};//坐标系，空间向量
    draw_geom(&scn, mjGEOM_SPHERE, size, pos, mat, color);
    /*--------3D绘制--------*/

    /*--------速度跟踪--------*/
    auto lin_vel = get_sensor_data(m, d, "base_lin_vel");
    auto base_pos = get_sensor_data(m, d, "base_pos");
    for (int i = 0; i < 3; i++) {
      from[i] = base_pos[i];
      to[i] = base_pos[i] + lin_vel[i] * 5;
    }
    from[2] += 0.5;
    to[2] += 0.5;
    float color3[4] = {0.3, 0.6, 0.3, 0.9};
    draw_arrow(&scn, from, to, 0.1, color3);
    /*--------速度跟踪--------*/

    mjr_render(viewport, &scn, &con);

    /*--------2D绘制--------*/
    mjr_text(mjFONT_NORMAL, "Albusgive", &con, 0, 0.9, 1, 0, 1);
    mjrRect viewport2 = {50, 100, 50, 50};
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, "github", "Albusgive",
                &con);
    mjr_rectangle(viewport2, 0.5, 0, 1, 0.6);
    mjrRect viewport3 = {100, 200, 150, 50};
    mjr_label(viewport3, mjFONT_NORMAL, "Albusgive", 0, 1, 1, 1, 0, 0, 0, &con);
    /*--------2D绘制--------*/

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return 1;
}
