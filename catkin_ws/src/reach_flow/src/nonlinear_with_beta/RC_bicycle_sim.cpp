//#include "../flowstar-template/Continuous.h"
#include "../flowstar/Continuous.h"

using namespace flowstar;
using namespace std;


int main()
{
	unsigned int numVars = 3;

	int v_id = stateVars.declareVar("v");
	int noise_id = stateVars.declareVar("noise");
	//int psi_id = stateVars.declareVar("psi");
	//int beta_id = stateVars.declareVar("beta");
	//int v_id = stateVars.declareVar("v");
	int t_id = stateVars.declareVar("t");



	// define the dynamics
	Expression_AST<Real> ode_expression_v("1");
	//Expression_AST<Real> ode_expression_y("v * sin(psi+beta)");
	//Expression_AST<Real> ode_expression_psi("v * sin(beta)/0.12");
	Expression_AST<Real> ode_expression_noise("0");
	//Expression_AST<Real> ode_expression_v("0");
	Expression_AST<Real> ode_expression_t("1");



	vector<Expression_AST<Real> > ode_rhs(numVars);
	ode_rhs[v_id] = ode_expression_v;
	ode_rhs[noise_id] = ode_expression_noise;
	//ode_rhs[psi_id] = ode_expression_psi;
	//ode_rhs[beta_id] = ode_expression_beta;
	//ode_rhs[v_id] = ode_expression_v;
	ode_rhs[t_id] = ode_expression_t;



	Deterministic_Continuous_Dynamics dynamics(ode_rhs);



	// set the reachability parameters
	Computational_Setting setting;

	// set the stepsize and the order
	setting.setFixedStepsize(0.1, 2);
//	setting.setFixedStepsize(0.04, 5, 8);
//	setting.setAdaptiveStepsize(0.01, 0.04, 5);

	// set the time horizon
	setting.setTime(0.5);

	// set the cutoff threshold
	setting.setCutoffThreshold(1e-8);

	// set the queue size for the symbolic remainder, it is 0 if symbolic remainder is not used
	setting.setQueueSize(100);

	// print out the computation steps
	setting.printOn();

	// set up the remainder estimation
	Interval I(-0.01, 0.01);
	vector<Interval> remainder_estimation(numVars, I);
	setting.setRemainderEstimation(remainder_estimation);

	// call this function when all of the parameters are defined
	setting.prepare();


	// define the initial set which is a box
	Interval init_v(-0.001, 0.001), init_noise(-1, 1), init_t;

	vector<Interval> initial_box(numVars);
	initial_box[v_id] = init_v;
	initial_box[noise_id] = init_noise;
	//initial_box[psi_id] = init_psi;
	//initial_box[beta_id] = init_beta;
	//initial_box[v_id] = init_v;
	initial_box[t_id] = init_t;

	Flowpipe initialSet(initial_box);


	// empty unsafe set
	vector<Constraint> unsafeSet;


	/*
	 * The structure of the class Result_of_Reachability is defined as below:
	 * nonlinear_flowpipes: the list of computed flowpipes
	 * tmv_flowpipes: translation of the flowpipes, they will be used for further analysis
	 * fp_end_of_time: the flowpipe at the time T
	 */
	Result_of_Reachability result;

	// run the reachability computation
	clock_t begin, end;
	begin = clock();

	dynamics.reach(result, setting, initialSet, unsafeSet);
//	dynamics.reach(result, setting, result.fp_end_of_time, unsafeSet);
//	dynamics.reach(result, setting, result.fp_end_of_time, unsafeSet);

	end = clock();
	printf("time cost: %lf\n", (double)(end - begin) / CLOCKS_PER_SEC);

	// flowpipes should be translated to single Taylor model vectors before plotting
	result.transformToTaylorModels(setting);


	Plot_Setting plot_setting;
	plot_setting.printOn();
	plot_setting.setOutputDims(t_id, noise_id);
	plot_setting.plot_2D_interval_MATLAB("RC_bicycle", result);
//	plot_setting.plot_2D_grids_GNUPLOT("test", 10, result);

	return 0;
}

