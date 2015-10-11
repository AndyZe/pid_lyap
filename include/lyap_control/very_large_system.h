#include "ros/ros.h"

// Header for 'plant_msg.msg'
#include "lyap_control/plant_msg.h"

// Header for controller_msg.msg
#include "lyap_control/controller_msg.h"

/////////////////////////////////////////////
// User-defined variables -- Make changes here.
/////////////////////////////////////////////

static const int num_states=14;
static const int num_inputs=15;

// Initial conditions
static const double x_IC [num_states] = {.3, .4, .5, .6, .7, .8, .9, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7};
static const double t_IC = 0.0;
static const double setpoint [num_states] = {1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.};

double delta_t = 0.000001; // control period in seconds

// Global so it can be passed from the callback fxn to main
static double u[num_inputs] = {0.0, 0.0};


/////////////////////////////////////////////
// Functions
/////////////////////////////////////////////

void chatterCallback(const lyap_control::controller_msg& u_msg);
