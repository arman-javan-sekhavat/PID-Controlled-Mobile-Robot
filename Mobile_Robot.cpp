#include <cstdio>
#include <cstring>
#include <cmath>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <iostream>



// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context


bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;



void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {

    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}



void mouse_button(GLFWwindow* window, int button, int act, int mods) {

    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}


void mouse_move(GLFWwindow* window, double xpos, double ypos) {

    if (!button_left && !button_middle && !button_right) {
        return;
    }

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;


    int width, height;
    glfwGetWindowSize(window, &width, &height);


    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)) == GLFW_PRESS;
    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;


    mjtMouse action;
    if (button_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (button_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else {
        action = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}



void scroll(GLFWwindow* window, double xoffset, double yoffset) {

    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}



//****************************************************
//****************************************************
//****************************************************
//****************************************************
//****************************************************
//****************************************************
//****************************************************
//****************************************************
//****************************************************

const mjtNum PI = 3.141592653589;
const mjtNum s = 1.2958;


mjtNum x, x_d;
mjtNum y, y_d;
mjtNum theta_d;

mjtNum e_x = 0, e_y = 0, e_t = 0;
mjtNum ie_x = 0, ie_y = 0, ie_t = 0;
mjtNum de_x, de_y, de_t;

mjtNum e_x_old, e_y_old, e_t_old;

mjtNum PIDx, PIDy, PIDt;

mjtNum Y1, Y2;
mjtNum T1, T2;

mjtNum Kp = 300;
mjtNum Ki = 0;
mjtNum Kd = 70;


void controller(const mjModel* m, mjData* d) {

    PIDx = Kp * e_x + Ki * ie_x + Kd * de_x;
    PIDy = Kp * e_y + Ki * ie_y + Kd * de_y;
    PIDt = Kp * e_t + Ki * ie_t + Kd * de_t;

    d->ctrl[0] = +(s/2)*PIDt + Y1*PIDx + Y2*PIDy;
    d->ctrl[1] = -(s/2)*PIDt + Y1*PIDx + Y2*PIDy;
}



// main function
int main(int argc, const char** argv) {

    if (argc != 2) {
        std::printf(" USAGE:  basic modelfile\n");
        return 0;
    }

    // load and compile model
    char error[1000] = "Could not load binary model";
    if (std::strlen(argv[1]) > 4 && !std::strcmp(argv[1] + std::strlen(argv[1]) - 4, ".mjb")) {
        m = mj_loadModel(argv[1], 0);
    }
    else {
        m = mj_loadXML(argv[1], 0, error, 1000);
    }
    if (!m) {
        mju_error("Load model error: %s", error);
    }

    // make data
    d = mj_makeData(m);


    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }


    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);


    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);


    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);


    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);


    // ********************************************************************************************
    // ********************************************************************************************
    // ********************************************************************************************
    // ********************************************************************************************


    mjcb_control = controller;



    while (!glfwWindowShouldClose(window)) {

        mjtNum simstart = d->time;


        while (d->time - simstart < 1.0 / 60.0) {


            if (d->time < 3) {
                x_d = 3;
                y_d = d->time;
                theta_d = PI / 2;
            }
            else if (d->time > 3 && d->time < 13) {
                theta_d = PI / 2 + (d->time - 3) * (PI / 10);
                x_d = 3 * cos(theta_d - PI / 2);
                y_d = 3 + 3 * sin(theta_d - PI / 2);
            }
            else if (d->time > 13 && d->time < 19) {
                x_d = -3;
                y_d = 3 - (d->time-13);
                theta_d = (3*PI) / 2;
            }
            else if (d->time > 19) {
                theta_d = (3*PI) / 2 + (d->time - 19) * (PI / 10);
                x_d = 3 * cos(theta_d - PI / 2);
                y_d = -3 + 3 * sin(theta_d - PI / 2);
            }

            //----------------------------------------------------------------

            T1 = cos(theta_d);
            T2 = sin(theta_d);

            Y1 = d->sensordata[3];
            Y2 = d->sensordata[4];

            x = d->sensordata[0];
            y = d->sensordata[1];
            
            //----------------------------------------------------------------

            e_x_old = e_x;
            e_y_old = e_y;
            e_t_old = e_t;

            e_x = x_d - x;
            e_y = y_d - y;
            e_t = Y1 * T2 - Y2 * T1;

            ie_x += e_x * m->opt.timestep;
            ie_y += e_y * m->opt.timestep;
            ie_t += e_t * m->opt.timestep;

            de_x = (e_x - e_x_old) / m->opt.timestep;
            de_y = (e_y - e_y_old) / m->opt.timestep;
            de_t = (e_t - e_t_old) / m->opt.timestep;

            mj_step(m, d);

        }

        // ********************************************************************************************
        // ********************************************************************************************
        // ********************************************************************************************
        // ********************************************************************************************



        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);


        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);


        glfwSwapBuffers(window);


        glfwPollEvents();
    }


    mjv_freeScene(&scn);
    mjr_freeContext(&con);


    mj_deleteData(d);
    mj_deleteModel(m);


#if defined(__APPLE__)  defined(_WIN32)
    glfwTerminate();
#endif

    return 0;
}