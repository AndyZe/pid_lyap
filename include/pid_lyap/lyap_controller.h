#pragma once

// Using ublas for vector/matrix multiplication
#include <boost/numeric/ublas/vector.hpp>
typedef boost::numeric::ublas::vector <double> ublas_vector;
#include <boost/numeric/ublas/matrix.hpp>

// Using to check for NaN
#include <boost/math/special_functions/fpclassify.hpp>

#include "ros/ros.h"

// Header for 'plant_msg.msg'
#include "pid_lyap/plant_msg.h"
// Header for controller_msg.msg
#include "pid_lyap/controller_msg.h"

/////////////////////////////////////////////////////////////////////////////
// These variables need to be global b/c a ROS callback only takes 1 argument
/////////////////////////////////////////////////////////////////////////////

static double t=0; // time will be updated by listening to the 'plant' ROS topic
static double V=0; //current Lyapunov value
static double V_initial=0;
static int first_callback=1; // 1 signals that the callback has not been run yet. Triggers setup calcs

// Read the size of a plant_msg
pid_lyap::plant_msg temp_plant_msg; // Just to read the msg size
const static int num_states = temp_plant_msg.x.size();

// Message variable for the control effort message
pid_lyap::controller_msg  u_msg;
// Read the size of a 'controller' message
const static int num_inputs = u_msg.u.size();

ublas_vector x(num_states);
ublas_vector setpoint(num_states);
ublas_vector u(num_inputs);

///////////////////////////////////////////////////////////////////////////////
// User-defined parameters
///////////////////////////////////////////////////////////////////////////////


// Edit this header to define your own dynamic system:
#include "pid_lyap/model_definition.h"


/////////////////////////////////////////////////////////////////////////////
// Functions
/////////////////////////////////////////////////////////////////////////////

// The main callback to calculate u
void chatterCallback(const pid_lyap::plant_msg& msg);

void initial_error_check(const pid_lyap::plant_msg& msg);

// Calculate dx_dot_du and open_loop_dx_dt
void calculate_dx_dot_du(boost::numeric::ublas::matrix<double> &dx_dot_du, ublas_vector & open_loop_dx_dt);

// Calculate V_initial, V, and V_dot_target
void calculate_V_and_damping(double &V_dot_target);

// Calculate a stabilizing control effort
void calculate_u(ublas_vector &D, ublas_vector &open_loop_dx_dt, const double &V_dot_target, boost::numeric::ublas::matrix<double> &dx_dot_du);
