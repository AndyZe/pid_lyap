// This file simulates a simple dynamic system, publishes its state,
// and subscribes to a 'control_effort' topic. The control effort is used
// to stabilize the plant.

#include "lyap_control/very_large_system.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "plant");
  
  ros::NodeHandle plant_node;

  // Declare a new message variable
  lyap_control::plant_msg  msg;

  // Initial conditions -- these were defined in the header file
  for (int i=0; i<num_states; i++)
  {
    msg.x[i] = x_IC[i];
    msg.setpoint[i] = setpoint[i];
  }
  msg.t = t_IC;

  // Publish a plant.msg
  ros::Publisher chatter_pub = plant_node.advertise<lyap_control::plant_msg>("state", 1);

  // Subscribe to "control_effort" topic to get a controller_msg.msg
  ros::Subscriber sub = plant_node.subscribe("control_effort", 1, chatterCallback );
  
  double x_dot [num_states] = {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};

  ros::Rate loop_rate(1/delta_t); // Control rate in Hz


  while (ros::ok())
  {
    ROS_INFO("x1: %f  x2: %f", msg.x[0], msg.x[1]);

    chatter_pub.publish(msg);

    // Update the plant.
    x_dot[0] = 0.5*(u[0]-msg.x[0]);
    x_dot[1] = msg.x[1]-u[1];
    x_dot[2] = msg.x[2]*msg.x[3]+u[2]+u[3];
    x_dot[3] = u[2]*msg.x[2]*msg.x[3]+u[4];
    x_dot[4] = -u[5]*msg.x[2]*msg.x[4];
    x_dot[5] = u[6]-msg.x[5];
    x_dot[6] = 0.5*(u[7]-4.*msg.x[6]);
    x_dot[7] = 0.333*(u[8]-9.*msg.x[7]);
    x_dot[8] = 0.25*(u[9]-16.*msg.x[8]);
    x_dot[9] = 0.2*(u[10]-25.*msg.x[9]);
    x_dot[10] = 0.167*(u[11]-36.*msg.x[10]);
    x_dot[11] = 0.143*(u[12]-49.*msg.x[11]);
    x_dot[12] = msg.x[13]+u[14];
    x_dot[13] = -msg.x[12]+(1-msg.x[12]*msg.x[12])*msg.x[13]+u[13];

    msg.x[0] = msg.x[0]+x_dot[0]*delta_t;
    msg.x[1] = msg.x[1]+x_dot[1]*delta_t;
    msg.x[2] = msg.x[2]+x_dot[2]*delta_t;
    msg.x[3] = msg.x[3]+x_dot[3]*delta_t;
    msg.x[4] = msg.x[4]+x_dot[4]*delta_t;
    msg.x[5] = msg.x[5]+x_dot[5]*delta_t;
    msg.x[6] = msg.x[6]+x_dot[6]*delta_t;
    msg.x[7] = msg.x[7]+x_dot[7]*delta_t;
    msg.x[8] = msg.x[8]+x_dot[8]*delta_t;
    msg.x[9] = msg.x[9]+x_dot[9]*delta_t;
    msg.x[10] = msg.x[10]+x_dot[10]*delta_t;
    msg.x[11] = msg.x[11]+x_dot[11]*delta_t;
    msg.x[12] = msg.x[12]+x_dot[12]*delta_t;
    msg.x[13] = msg.x[13]+x_dot[13]*delta_t;
    
    msg.t = msg.t+delta_t;

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

// Callback when something is published on 'control_effort'
void chatterCallback(const lyap_control::controller_msg& u_msg)
{
  //ROS_INFO("I heard: %f %f", u_msg.u[0], u_msg.u[1]);

  // Define the stabilizing control effort
  for (int i=0; i< num_inputs; i++)
    u[i] = u_msg.u[i];
}
