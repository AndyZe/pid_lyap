// The state space definition-- the dynamic equations of the model.
// Calculates dx/dt
// model_definition reads u sees u b/c it's a global variable, it can't be an argument
void model_definition(const ublas_vector &x, ublas_vector &dxdt, const double t)
{
  dxdt[0] = 0.5*(u[0]-x[0]);
  dxdt[1] = x[1]-u[1];
  dxdt[2] = x[2]*x[3]+u[2]+u[3];
  dxdt[3] = u[2]*x[2]*x[3]+u[4];
  dxdt[4] = -u[5]*x[2]*x[4];
  dxdt[5] = u[6]-x[5];
  dxdt[6] = 0.5*(u[7]-4.*x[6]);
  dxdt[7] = 0.33*(u[8]-9.*x[7]);
  dxdt[8] = 0.25*(u[9]-16.*x[8]);
  dxdt[9] = 0.2*(u[10]-25.*x[9]);
  dxdt[10] = 0.167*(u[11]-36.*x[10]);
  dxdt[11] = 0.143*(u[12]-49.*x[11]);
  dxdt[12] = x[13]+u[14];
  dxdt[13] = -x[12]+(1-x[12]*x[12])*x[13]+u[13];
}
