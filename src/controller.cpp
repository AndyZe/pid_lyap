
// Subscribe to a topic about the state of a dynamic system and calculate feedback to
// stabilize it.
// Should run at a faster loop rate than the plant.


#include "lyap_control/controller_header.h"


/////////////////////////////////////////////////////////////////////////////////////////
// User-defined parameters
/////////////////////////////////////////////////////////////////////////////////////////

static const double V_dot_target_initial= -10000.0;

// Control frequency in Hz. Should be > the msg frequency that's published by the plant
static const double rate = 1000000.0;

static const double high_saturation_limit [] = {50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50.};
static const double low_saturation_limit []= {-50., -50., -50., -50., -50., -50., -50., -50., -50., -50., -50., -50., -50., -50., -50.};

// Parameter for V2 step location, gamma
static const double g = 1.0;

// Threshold for switching to V2
static const double switch_threshold = .01;


////////////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
  ROS_INFO("Check the num. of states and num. of inputs.");
  ROS_INFO("These can be adjusted in 'controller_msg.msg' and 'plant_msg.msg' files."); 
  ROS_INFO("num_states: %i num_inputs: %i",num_states,num_inputs);
  
  ROS_INFO(" ");
  ROS_INFO("Also check the parameters of the controller at the top of lyap_control/src/controller.cpp");

  ros::init(argc, argv, "controller");

  ros::NodeHandle controller_node;

  // Publish on "control_effort" topic
  ros::Publisher chatter_pub = controller_node.advertise<lyap_control::controller_msg>("control_effort", 1);

  // Subscribe to "state" topic
  ros::Subscriber sub = controller_node.subscribe("state", 1, chatterCallback );

  ros::Rate loop_rate(rate); // Control frequency in Hz

  while (ros::ok())
  {
    ros::spinOnce();

    // Publish the stabilizing control effort
    chatter_pub.publish(u_msg);

    loop_rate.sleep();
  }

  return 0;
}


///////////////////////////////////////////////////////////////
// The main callback where a stabilizing u is calculated.
// This is where the magic happens.
///////////////////////////////////////////////////////////////
void chatterCallback(const lyap_control::plant_msg& msg)
{
  //ROS_INFO("I heard x: %f %f", msg.x[0]);

  if ( first_callback )
  {
    initial_error_check(msg);
  }

  //Convert the message into the boost type we need
  for (int i=0; i<num_states; i++)
  {
    x[i]=msg.x[i];
    setpoint[i] = msg.setpoint[i];
  }

  // dx_dot_du is calculated
  boost::numeric::ublas::matrix<double> dx_dot_du (num_inputs, num_states);
  ublas_vector open_loop_dx_dt(x);
  calculate_dx_dot_du( dx_dot_du, open_loop_dx_dt );
  //ROS_INFO( "dxdotdu: %f %f %f %f", dx_dot_du(0,0), dx_dot_du(0,1), dx_dot_du(1,0), dx_dot_du(1,1) );

  //Calc D's: the sum of x[i]*dx_dot[i]/du for all u's
  ublas_vector D(num_inputs);

  D.clear();

  for (int i=0; i<num_inputs; i++)
    for (int j=0; j<num_states; j++)
      // D[i] = sum( x[j]*df[j]/du[i])
      D[i] = D[i]+( msg.x[j]-setpoint[j] )*dx_dot_du(i,j);
  //ROS_INFO("D[0]: %f D[1]: %f", D[0], D[1]);

  // Calculate V_initial, V, and V_dot_target
  double V_dot_target;
  calculate_V_and_damping(V_dot_target);

  //Calc u to force V_dot<0
  calculate_u(D, open_loop_dx_dt, V_dot_target, dx_dot_du);

  //Check control effort saturation
  for (int i=0; i<num_inputs; i++)
  {
    if ( u[i] < low_saturation_limit[i] )
    {
      u[i] = low_saturation_limit[i];
    }
    if ( u[i] > high_saturation_limit[i] )
    {
      u[i] = high_saturation_limit[i];
    }
  }

  // Stuff the message to be published
  for (int i=0; i<num_inputs; i++)
    u_msg.u[i] = u[i];

  //ROS_INFO("Calculated u: %f", u_msg.u[0]);
}

////////////////////////////////////////////////////////////////////
// Initial error check
////////////////////////////////////////////////////////////////////
void initial_error_check(const lyap_control::plant_msg& msg)
{
  if ( V_dot_target_initial >= 0 )
  {
    ROS_ERROR("V_dot_target_initial must be negative. Change it in controller_parameters.h.");
    ros::shutdown();
  }

  if ( (sizeof(high_saturation_limit)+sizeof(low_saturation_limit)) / sizeof(*high_saturation_limit) != 2*num_inputs )
  {
    ROS_ERROR("Check your saturation limit definitions. There are too many or too few values in one of the arrays. They must match the # of inputs to the system.");
    ros::shutdown();
  }

  for (int i=0; i<num_inputs; i++)
    if ( low_saturation_limit[i] >= high_saturation_limit[i] )
    {
      ROS_ERROR("The 'low saturation limit' is higher than 'high saturation limit.' Change them in controller_parameters.h.");
      ros::shutdown();
    };

  if ( msg.setpoint.size() != num_states )
  {
    ROS_ERROR("The published setpoint's length does not equal # of states. Check the data which is published by the plant.");
    ros::shutdown();
  }
}

////////////////////////////////////////////////////////////////////
// Calculate dx_dot_du and open_loop_dx_dt
////////////////////////////////////////////////////////////////////
void calculate_dx_dot_du( boost::numeric::ublas::matrix<double> &dx_dot_du, ublas_vector &open_loop_dx_dt )
{
  /////////////////////////////////////////////////////////////////////////
  // Base case, open loop. Compare against this.
  /////////////////////////////////////////////////////////////////////////

  // Clear u since this is open loop
  u.clear();

  model_definition(x,open_loop_dx_dt,t);

  //////////////////////////////////////////////////////////////////////////
  // Now apply a u and see the result
  //////////////////////////////////////////////////////////////////////////

  ublas_vector closed_loop_dx_dt(num_states);

  for ( int i=0; i<num_inputs; i++ )
  {
    u.clear();
    u[i] = 1.0;
    model_definition(x,closed_loop_dx_dt,t); //See how this u affects the ode's. Closed_loop_dx_dt is returned
    for ( int j=0; j<num_states; j++ )
    {
       dx_dot_du(i,j) = closed_loop_dx_dt[j]-open_loop_dx_dt[j]; // divide by u[i]=1.0 ==> neglected for speed
    }
  }
}

////////////////////////////////////////////////////////////////////
// Calculate V_initial, V, and V_dot_target
////////////////////////////////////////////////////////////////////
void calculate_V_and_damping(double &V_dot_target)
{
  //Calculate V given the incoming msg.x
  V = 0.5*inner_prod(x-setpoint,x-setpoint);
  ROS_INFO("V: %f", V);
  // Store this V as V_initial if this is the first time through.
  if ( first_callback )
  {
     first_callback=0;
     V_initial = V;
  }

  //Adjust the Lyapunov damping
  //V_dot_target = (V(epoch)/V(1))^2*V_dot_target_initial;
  V_dot_target = pow((V/V_initial),2.0)*V_dot_target_initial;
}

////////////////////////////////////////////////////////////////////
// Calculate a stabilizing control effort
////////////////////////////////////////////////////////////////////
void calculate_u(ublas_vector &D, ublas_vector &open_loop_dx_dt, const double &V_dot_target, boost::numeric::ublas::matrix<double> &dx_dot_du)
{
  u.clear();

  // Find the largest |D_i|
  // It will be the base for the u_i calculations.
  // If all D_i's are ~0, use V2.
  int largest_D_index = index_norm_inf(D);

  if ( fabs(D[largest_D_index]) > switch_threshold ) // Use V1
  {
    ublas_vector P = element_prod(x-setpoint,open_loop_dx_dt); // x_i*x_i_dot

    // Start with the u that has the largest effect
    if ( fabs( D[largest_D_index] + ( sum( element_prod(D,D) ) - pow(D[largest_D_index],2.0) )/D[largest_D_index]) > 0.0001 ) 
      u(largest_D_index) = (V_dot_target-sum(P)) / 
        ( D[largest_D_index] + ( sum( element_prod(D,D) ) - pow(D[largest_D_index],2.0) )/D[largest_D_index] );

    // Now scale the other u's (if any) according to u_max*Di/D_max
    for ( int i=0; i<num_inputs; i++ )
      if ( i != largest_D_index ) // Skip this entry, we already did it
      {
        //if ( fabs(D[i]) > 0.1*fabs(D[largest_D_index]) ) // If this D has a significant effect. Otherwise this u will remain zero
        //{
          u[i] = u[largest_D_index]*D[i]/D[largest_D_index];
        //}
      }
  } // End of V1 calcs

  else // Use V2
  {

    //ublas_vector dV2_dx = (x-setpoint)*(0.9+0.1*sum( element_prod(x-setpoint,x-setpoint) ));

    ublas_vector dV2_dx = (x-setpoint)*(0.9+0.1*tanh(g)+0.05*sum( element_prod(x-setpoint,x-setpoint) )*pow(cosh(g),-2));

    // The first entry in dV2_dx is unique because the step is specified in x1, so replace the previous
    dV2_dx[0] = (x[0]-setpoint[0])*(0.9+0.1*tanh(g))+
	0.05*sum(element_prod(x-setpoint,x-setpoint))*(pow((x[0]-setpoint[0]),-1)*tanh(g)+(x[0]-setpoint[0])*pow(cosh(g),-2));

    //MATLAB equivalent:
    //P_star = dV2_dx.*f_at_u_0(1:num_states);

    ublas_vector P_star = element_prod(dV2_dx, open_loop_dx_dt);

    //MATLAB equivalent:
    //D_star = zeros(num_inputs,1);
    //for i=1:num_inputs
    //  for j=1:num_states
    //    D_star(i) = D_star[i]+dV2_dx(j)*dx_dot_du(i,j);
    //  end
    //end

    ublas_vector D_star(num_inputs);


    for (int i=0; i<num_inputs-1; i++)
      for (int j=0; j<num_inputs-1; j++)
        D_star[i] = D_star[i]+dV2_dx[j]*dx_dot_du(i,j);


    // The first input is unique.
    // MATLAB equivalent:
    //u(epoch,1) = (V_dot_target-sum(P_star)) / ...
    //( D_star(1) + (sum( D_star.*D_star )- D_star(1)^2) /...
    //D_star(1) );

    u[0] = (V_dot_target-sum(P_star)) / 
      ( D_star[0] + (sum( element_prod(D_star,D_star) )-pow(D_star[0],2)) / D_star[0] );

     // For the other inputs
     // MATLAB equivalent:
     //for i=2:num_inputs
     //  u(epoch,i) = u(epoch,1)*D_star(i)/D_star(1);
     //end

     for (int i=1; i<num_inputs; i++)
     {
       //u[i] = u[0]*D_star[i]/D_star(0);
       u[i] = u[0]*D_star[i]/0.0;
     }

     // Check for NaN (caused by D_star(1)==0).
     // It means the system is likely uncontrollable.
     // MATLAB equivalent:
     //if ~isfinite( u(epoch,:) )
     //  u(epoch,:)= zeros(num_inputs,1);
     //end
  
     for (int i=0; i<num_inputs; i++)
     {
       if ( (boost::math::isnan)(u[i]) )
       {
         u.clear(); // Reset u to zeros
         //ROS_INFO("isnan");
         break; // Short circuit, could save time for long u vectors
       }
     }
  } // End of V2 calcs
}
