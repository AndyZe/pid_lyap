#include "ros/ros.h"

// Header for 'plant_msg.msg'
#include "pid_lyap/plant_msg.h"

// Header for controller_msg.msg
#include "pid_lyap/controller_msg.h"

/////////////////////////////////////////////
// Variables -- Make changes here.
/////////////////////////////////////////////

static const int num_states=1;
static const int num_inputs=1;

// Initial conditions
static const double x_IC [num_states] = {4.7};
static const double t_IC = 0.0;
static const double setpoint [num_states] = {-1.0};

double delta_t = 0.0001; // control period in seconds

// Global so it can be passed from the callback fxn to main
static double u[num_inputs] = {0};


/////////////////////////////////////////////
// Functions
/////////////////////////////////////////////

void chatterCallback(const pid_lyap::controller_msg& u_msg);
