// The state space definition-- the dynamic equations of the model.
// Calculates dx/dt
// model_definition reads u sees u b/c it's a global variable, it can't be an argument
void model_definition(const ublas_vector &x, ublas_vector &dxdt, const double t)
{
  dxdt[0] = x[0]+u[0];
  dxdt[1] = x[1]-100.*u[1];
}
