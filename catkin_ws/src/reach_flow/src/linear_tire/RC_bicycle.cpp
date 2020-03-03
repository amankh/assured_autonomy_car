#include "../flowstar/Continuous.h"

using namespace flowstar;
using namespace std;

//I_z = 0.0558
//l_f = 0.137
//l_r = 0.12

int main(int argc, char *argv[])
{
	unsigned int numVars = 11;

	int x_id = stateVars.declareVar("x");
	int y_id = stateVars.declareVar("y");
	int psi_id = stateVars.declareVar("psi");
        int psidot_id = stateVars.declareVar("psidot");
        int Fxf_id = stateVars.declareVar("Fxf");
        int Fxr_id = stateVars.declareVar("Fxr");
        int Fyf_id = stateVars.declareVar("Fyf");
        int Fyr_id = stateVars.declareVar("Fyr");
        int beta_id = stateVars.declareVar("beta");
	int v_id = stateVars.declareVar("v");
	int t_id = stateVars.declareVar("t");

        double x_0_min = atof(argv[1]);
        double x_0_max = atof(argv[2]);
        double y_0_min = atof(argv[3]);
        double y_0_max = atof(argv[4]);
        double psi_0_min = atof(argv[5]);
        double psi_0_max = atof(argv[6]);
        double psidot_0_min = atof(argv[7]);
        double psidot_0_max = atof(argv[8]);
        double Fxf_0_min = atof(argv[9]);
        double Fxf_0_max = atof(argv[10]);
        double Fxr_0_min = atof(argv[11]);
        double Fxr_0_max = atof(argv[12]);
        double Fyf_0_min = atof(argv[13]);
        double Fyf_0_max = atof(argv[14]);
        double Fyr_0_min = atof(argv[15]);
        double Fyr_0_max = atof(argv[16]);
        double beta_0_min = atof(argv[17]);
        double beta_0_max = atof(argv[18]);
        double v_0_min = atof(argv[19]);
        double v_0_max = atof(argv[20]);
        string formula = argv[21];
        

	// define the dynamics
	Expression_AST<Real> ode_expression_x("v * cos(psi+beta)");
	Expression_AST<Real> ode_expression_y("v * sin(psi+beta)");
	Expression_AST<Real> ode_expression_psi("psidot");
	//Expression_AST<Real> ode_expression_vx("psidot * ydot +1/m * (F_xr - F_yf * sin(delta))");
	//Expression_AST<Real> ode_expression_vy("-psidot * vx + 1/m * (F_xr*cos(delta)+F_yr)");
        Expression_AST<Real> ode_expression_psidot("-2.15*Fyr+2.45*Fyf");//1/I_z * (l_f*F_yf-l_r*F_yr)
        //Expression_AST<Real> ode_expression_psidot("0");
        Expression_AST<Real> ode_expression_Fxf("0");
        Expression_AST<Real> ode_expression_Fxr("0");
        Expression_AST<Real> ode_expression_Fyf("0");
        Expression_AST<Real> ode_expression_Fyr("0");
        Expression_AST<Real> ode_expression_beta("0");
        Expression_AST<Real> ode_expression_v("0");
	Expression_AST<Real> ode_expression_t("1");



	vector<Expression_AST<Real> > ode_rhs(numVars);
	ode_rhs[x_id] = ode_expression_x;
	ode_rhs[y_id] = ode_expression_y;
	ode_rhs[psi_id] = ode_expression_psi;
        ode_rhs[psidot_id] = ode_expression_psidot;
        ode_rhs[Fxf_id] = ode_expression_Fxf;
        ode_rhs[Fxr_id] = ode_expression_Fxr;
        ode_rhs[Fyf_id] = ode_expression_Fyf;
        ode_rhs[Fyr_id] = ode_expression_Fyr; 
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
	setting.setTime(0.1);

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
	//Interval init_x(1, 1.01), init_y(-0.01, 0.01), init_psi(-0.01, 0.01),
	//		init_psidot(-0.001, 0.001), init_Fxf(0.0, 0,0), init_Fxr(0,0, 0.0), init_Fyf(0.0, 0.01), init_Fyr(0.0, 0.01), init_beta(0.0, 0.01), init_v(0.6, 0.61), init_t;
	Interval init_x(x_0_min, x_0_max), init_y(y_0_min, y_0_max), init_psi(psi_0_min, psi_0_max), init_psidot(psidot_0_min, psidot_0_max), init_Fxf(Fxf_0_min, Fxf_0_max), init_Fxr(Fxr_0_min, Fxr_0_max),
			init_Fyf(Fyf_0_min, Fyf_0_max), init_Fyr(Fyr_0_min, Fyr_0_max), init_beta(beta_0_min, beta_0_max), init_v(v_0_min, v_0_max), init_t;

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
        //Constraint cons1("-0.79+2*x+2*y-x*x-y*y");
        //Constraint cons1("1.41-x+y");
        //unsafeSet.push_back(cons1);

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

