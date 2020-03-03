#include "../flowstar/Continuous.h"

using namespace flowstar;
using namespace std;


int main(int argc, char *argv[])
{
	unsigned int numVars = 6;

	int x_id = stateVars.declareVar("x");
	int y_id = stateVars.declareVar("y");
	int psi_id = stateVars.declareVar("psi");
	int beta_id = stateVars.declareVar("beta");
	int v_id = stateVars.declareVar("v");
	int t_id = stateVars.declareVar("t");

        double x_0_min = atof(argv[1]);
        double x_0_max = atof(argv[2]);
        double y_0_min = atof(argv[3]);
        double y_0_max = atof(argv[4]);
        double psi_0_min = atof(argv[5]);
        double psi_0_max = atof(argv[6]);
        double beta_0_min = atof(argv[7]);
        double beta_0_max = atof(argv[8]);
        double v_0_min = atof(argv[9]);
        double v_0_max = atof(argv[10]);
        string formula = argv[11];
        



	// define the dynamics
	Expression_AST<Real> ode_expression_x("v * cos(psi+beta)");
        //Expression_AST<Real> ode_expression_x("v * cos(psi)");
	Expression_AST<Real> ode_expression_y("v * sin(psi+beta)");
        //Expression_AST<Real> ode_expression_y("v * sin(psi)");
	Expression_AST<Real> ode_expression_psi("v * sin(beta)/0.12");
	Expression_AST<Real> ode_expression_beta("0");
	Expression_AST<Real> ode_expression_v("0");
	Expression_AST<Real> ode_expression_t("1");



	vector<Expression_AST<Real> > ode_rhs(numVars);
	ode_rhs[x_id] = ode_expression_x;
	ode_rhs[y_id] = ode_expression_y;
	ode_rhs[psi_id] = ode_expression_psi;
	ode_rhs[beta_id] = ode_expression_beta;
	ode_rhs[v_id] = ode_expression_v;
	ode_rhs[t_id] = ode_expression_t;



	Deterministic_Continuous_Dynamics dynamics(ode_rhs);



	// set the reachability parameters
	Computational_Setting setting;

	// set the stepsize and the order
	setting.setFixedStepsize(0.1, 3);
//	setting.setFixedStepsize(0.04, 5, 8);
//	setting.setAdaptiveStepsize(0.01, 0.04, 5);

	// set the time horizon
	setting.setTime(0.5);

	// set the cutoff threshold
	setting.setCutoffThreshold(1e-8);

	// set the queue size for the symbolic remainder, it is 0 if symbolic remainder is not used
	setting.setQueueSize(100);

	// print out the computation steps
	setting.printOff();

	// set up the remainder estimation
	Interval I(-0.01, 0.01);
	vector<Interval> remainder_estimation(numVars, I);
	setting.setRemainderEstimation(remainder_estimation);

	// call this function when all of the parameters are defined
	setting.prepare();


	// define the initial set which is a box
	//Interval init_x(-0.01, 0.01), init_y(-0.01, 0.01), init_psi(-0.01, 0.01),
	//		init_beta(-0.00466938241262, 0.00466938241262), init_v(-0.01, 0.01), init_t;
	Interval init_x(x_0_min, x_0_max), init_y(y_0_min, y_0_max), init_psi(psi_0_min, psi_0_max),
			init_beta(beta_0_min, beta_0_max), init_v(v_0_min, v_0_max), init_t;

	vector<Interval> initial_box(numVars);
	initial_box[x_id] = init_x;
	initial_box[y_id] = init_y;
	initial_box[psi_id] = init_psi;
	initial_box[beta_id] = init_beta;
	initial_box[v_id] = init_v;
	initial_box[t_id] = init_t;

	Flowpipe initialSet(initial_box);


	// empty unsafe set
	vector<Constraint> unsafeSet;
        Constraint cons1(formula);
        //Constraint cons2("x_id-1");
        unsafeSet.push_back(cons1);
        //cout << formula << endl;
        //cons2.push_back();

	/*
	 * The structure of the class Result_of_Reachability is defined as below:
	 * nonlinear_flowpipes: the list of computed flowpipes
	 * tmv_flowpipes: translation of the flowpipes, they will be used for further analysis
	 * fp_end_of_time: the flowpipe at the time T
	 */
	Result_of_Reachability result;

	// run the reachability computation
	//clock_t begin, end;
	//begin = clock();

	dynamics.reach(result, setting, initialSet, unsafeSet);
//	dynamics.reach(result, setting, result.fp_end_of_time, unsafeSet);
//	dynamics.reach(result, setting, result.fp_end_of_time, unsafeSet);

	//end = clock();
	//printf("time cost: %lf\n", (double)(end - begin) / CLOCKS_PER_SEC);

	// flowpipes should be translated to single Taylor model vectors before plotting
	result.transformToTaylorModels(setting);
                

	Plot_Setting plot_setting;
	plot_setting.printOff();
	plot_setting.setOutputDims(x_id, y_id);
        //int indicator = 1;
	//int indicator = plot_setting.plot_2D_interval_MATLAB("RC_bicycle", result);//Qin
        plot_setting.plot_2D_interval_MATLAB("RC_bicycle", result);
        //printf("indicator", (double)(indicator));
        //if(indicator == 0)
        //{
        //    cout<<x_0_min<<endl;
        //    cout<<y_0_min<<endl; 
        //}
        //cout<<indicator<<endl;
//	plot_setting.plot_2D_grids_GNUPLOT("test", 10, result);

	return 0;
}

