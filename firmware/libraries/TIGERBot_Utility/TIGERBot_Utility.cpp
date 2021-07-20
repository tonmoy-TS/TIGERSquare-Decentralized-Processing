/**
 * @file    TIGERBot_Utility.cpp
 * @brief   Utility function implementations for the TIGERBot fleet.
 *
 * Provides barrier certificates (single-integrator and unicycle),
 * kinematic transformations (SI-to-unicycle), initialization checking,
 * automatic parking control, and polygon initial-condition generation
 * for multi-robot experiments on the TIGERSquare platform.
 *
 * Original Matlab version created by the Robotarium team (Georgia Tech).
 * TIGERSquare version created and modified by Victor Fernandez-Kim (2018).
 * Ported and extended for ESP8266 by Tonmoy Sarker (2021).
 */

//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
#include "TIGERBot_Utility.h"

// Constructors
TIGERBotUtility::TIGERBotUtility(){}

// Destructor
TIGERBotUtility::~TIGERBotUtility(){}

//-----------------------------------------------------------------
// Public Member Functions
//-----------------------------------------------------------------

/* ═══ Initialization check ═══════════════════════════════════════════════════════ */
				
bool TIGERBotUtility::create_is_initialized(const MatrixXf& x, const MatrixXf& ic,float pos_err,float rot_err){
/**
Creates a function to check for initialization.  The function returns whether 
all the agents have been initialized and those that have already finished.

Detailed Description
* PositionError - affects how close the agents are required to get to the desired position 
* RotationError - affects how close the agents are required to get to the desired rotation

Example Usage (MATLAB)
	initialization_checker = create_is_initialized('PositionError', 0.l,'RotationError', 0.01)
	[all_initialized, done_idxs] = initialization_checker(robot_poses, desired_poses)
**/
yield();	
	
    //float pos_err = 0.01; //PositionError
    //float rot_err = 0.5; //RotationError

        int M = x.rows(); //Serial.println(M);
		int N = x.cols();//Serial.println(N);
        int M_ic = ic.rows();//Serial.println(M_ic);
		int N_ic = ic.cols();//Serial.println(N_ic);
		int onTarget =0;

    if (M!=3)		Serial.printf("\n Row dimension of the states : %d, must be 3", M);
	if (M_ic!=3)	Serial.printf("\n Row dimension of conditions : %d, must be 3", M_ic);
    if (N_ic!=N)	Serial.printf("\nColumn dimension of states(:%d) and conditions(:%d) must be the same", N, N_ic);

	for(int i=0;i<N;i++){
		if (((x.col(i).head<2>() - ic.col(i).head<2>()).norm() <= pos_err) 
			&& (abs(wrap(x(2,i)-ic(2,i))) <= rot_err))
				onTarget = onTarget +1;
		else
			yield();
	}	
		if(N == onTarget){
				Serial.println("\nReached Initial positions.\n");
				return true;
		}
		else
				return false;
	
}


/* ═══ Barrier certificates ═══════════════════════════════════════════════════════ */

MatrixXf TIGERBotUtility::create_si_barrier_certificate(const MatrixXf& dxi, const MatrixXf& x,float safety_radius){
/**
Returns a single-integrator barrier certificate function f :R^2N * R^2N -> R^2N.
This function takes a 2 x N, 2 x N single-integrator velocity and state vector,
respectively, and returns a single-integrator velocity vector that does not induce
collisions in the agents.

Detailed Description
* BarrierGain - affects how quickly the agents can approach each other
* SafetyRadius - affects the distance the agents maintain

A good rule of thumb is to make SafetyRadius a bit larger than the agent
itself.
**/
yield();
    float gamma = 1e4; //BarrierGain
    //float safety_radius = 0.1;// default value 0.1

	/**
        %BARRIERCERTIFICATE Wraps single-integrator dynamics in safety barrier
        %certificates
        %   This function accepts single-integrator dynamics and wraps them in
        %   barrier certificates to ensure that collisions do not occur.  Note that
        %   this algorithm bounds the magnitude of the generated output to 0.1.
        %
        %   dx = BARRIERCERTIFICATE(dxi, x, safetyRadius)
        %   dx: generated safe, single-integrator inputs
        %   dxi: single-integrator dynamics
        %   x: States of the agents
        %   safetyRadius:  Size of the agents (or desired separation distance)
    **/
        int N = dxi.cols();

        Eigen::Matrix<float,2,Dynamic> dx(2,N); dx.fill(0);
        if(N < 2)
           dx = dxi;
        else
            dx=dx;

    //Generate constraints for barrier certificates based on the size of the safety radius
    int num_constraints = nchoosek(N, 2);
	
	/* QP variable map: H=G, f=g0, Aeq=CE, beq=ce0, A=CI, b=ci0, vnew=x */
	
 	MatrixXd H(2*N,2*N);			H = 2*H.setIdentity();	
	VectorXd f(2*N);
	MatrixXd Aeq(2*N,1);			Aeq.fill(0);
    VectorXd beq(1);				beq.fill(0);
    MatrixXf A(num_constraints,2*N);
    VectorXd b(num_constraints);
	VectorXd vnew(2*N); 

	int count = 0;
	for (int i=0;i<(N-1);i++){
        for(int j=i+1; j<N; j++){
            float mat_norm = (x.col(i).topRows(2)-x.col(j).topRows(2)).norm();
            float h = pow(mat_norm,2)- pow(safety_radius,2);
            A.row(count).col(2*i)   = -2*(x.col(i).row(0)-x.col(j).row(0));
            A.row(count).col(2*i+1) = -2*(x.col(i).row(1)-x.col(j).row(1));
            A.row(count).col(2*j)   =  2*(x.col(i).row(0)-x.col(j).row(0));
            A.row(count).col(2*j+1) =  2*(x.col(i).row(1)-x.col(j).row(1));
            b(count)= gamma*pow(h,3);
			count = count + 1;
		}
	}
	MatrixXd A_d = A.cast<double> ();
	MatrixXd dxi_ = dxi.cast<double>(); /* mutable double-precision copy of const dxi */

    Map<VectorXd> vhat(dxi_.data(),2*N); //vhat = reshape(dxi,2*N,1);%Column major
					//cout<<H<<endl;    
    f = -2*vhat;
	// Solve QP program generated earlier
	solve_quadprog(H, f, Aeq, beq, A_d.adjoint(), b, vnew);

    //Set robot velocities to new velocities
	MatrixXf vnew_float = vnew.cast <float> ();
    Map<MatrixXf> dx_(vnew_float.data(),2,N);   //dx = reshape(vnew, 2, N);
	
    return dx_;
}

MatrixXf TIGERBotUtility::create_uni_barrier_certificate(const MatrixXf& dxu, const MatrixXf& x,float safety_radius, float proj_dist){
/**
Returns a barrier certificate f: R^2N * R^3N -> R^2N that operates on unicycle
algorithms,preventing collisions.

Detailed Description

* BarrierGain - affects how quickly the robots can approach each other
* SafetyRadius - affects how far apart the robots must be
* ProjectionDistance - affects the utilized transformation to single-integrator dynamics

A good rule of thumb is to make the safety radius and projection distance each 1/2 of
the desired total distance for the agents to remain apart.You'll want their total to be
more than the diameter of the GRITSbot(i.e. 0.08 m).
// Example usage(Matlab)
   uni_barrier_cert = CREATE_UNI_BARRIER_CERTIFICATE('BarrierGain', 3,'SafetyRadius', 0.05, 'ProjectionDistance', 0.05)
   dxu_safe = uni_barrier_cert(dxu, robot_poses)
**/
	/* e^{\pi i} + 1 = 0 */
yield();

    float gamma = 5e3; //BarrierGain
    //float safety_radius = 0.5; //SafetyRadius
    //float proj_dist = 0.05; //ProjectionDistance or lambda
    float vel_mag_lim = 0.035; //VelocityMagnitudeLimit

    //[si_uni_dyn, uni_si_states] = create_si_to_uni_mapping('ProjectionDistance', proj_dist);
    //uni_si_dyn = create_uni_to_si_mapping('ProjectionDistance', proj_dist);

    int N = dxu.cols(); 									//cout<<"N:"<<N<<endl;
    if(N < 2)
        return dxu;
TRACE();
	Eigen::Matrix<float,2,Dynamic> xi(2,N); xi.fill(0);
	Eigen::Matrix<float,2,Dynamic> dxi(2,N); dxi.fill(0);

    // Shift to single integrator domain
    xi = create_si_to_uni_mapping_uni2siStates(x);			//cout<<"xi:\n"<<xi<<endl;
    dxi = create_uni_to_si_mapping_uni2siDyn(dxu, x);		//cout<<"dxi:\n"<< dxi<<endl;
    // Normalize velocities
    for(int i=0;i<N; i++){
        float norm_col = dxi.col(i).norm();
        if(norm_col > vel_mag_lim)
           dxi.col(i)= vel_mag_lim*(dxi.col(i).array()/norm_col);
    }
															//cout<<dxi<<endl;
TRACE();	
    // Generate constraints for barrier certificates based on the size of the safety radius 
    int num_constraints = nchoosek(N, 2); 					//cout<<"num_:"<<num_constraints<<endl;


	/* QP variable map: H=G, f=g0, Aeq=CE, beq=ce0, A=CI, b=ci0, vnew=x */
	
 	MatrixXd H(2*N,2*N);			H = 2*H.setIdentity();	
	VectorXd f(2*N);
	MatrixXd Aeq(2*N,1);			Aeq.fill(0);
    VectorXd beq(1);				beq.fill(0);
    MatrixXf A(num_constraints,2*N);
    VectorXd b(num_constraints);
	VectorXd vnew(2*N); 
TRACE();	
	int count = 0;
	for (int i=0;i<(N-1);i++){
        for(int j=i+1; j<N; j++){
            float mat_norm = (xi.col(i)-xi.col(j)).norm();
            float h = pow(mat_norm,2)-pow(safety_radius+2*proj_dist,2);
            A.row(count).col(2*i)   =  2*(xi.row(0).col(i)-xi.row(0).col(j));
            A.row(count).col(2*i+1) =  2*(xi.row(1).col(i)-xi.row(1).col(j));
            A.row(count).col(2*j)   = -2*(xi.row(0).col(i)-xi.row(0).col(j));
            A.row(count).col(2*j+1) = -2*(xi.row(1).col(i)-xi.row(1).col(j));
            b(count)= -gamma*pow(h,3);
			count = count + 1;
		}
	}
TRACE();
    A = -A;
    b = -b;
	
	MatrixXd A_d = A.cast<double> ();
    MatrixXd dxi_ = dxi.cast<double>(); /* mutable double-precision copy of const dxi */
    Map<VectorXd> vhat(dxi_.data(),2*N); //vhat = reshape(dxi,2*N,1);%Column major
					//cout<<H<<endl;    
    f = -2*vhat; 								//cout<<f<<endl;
TRACE();        
	// Solve QP program generated earlier
	solve_quadprog(H, f, Aeq, beq, A_d.adjoint(), b, vnew);
TRACE();	
    //Set robot velocities to new velocities
	MatrixXf vnew_float = vnew.cast <float> ();
    Map<MatrixXf> dxu_(vnew_float.data(),2,N);   //dxu = si_uni_dyn(reshape(vnew, 2, N), x);
    //cout<<"dxu_ from uni_barrier :" <<endl <<dxu_<<endl;
	dxu_ = create_si_to_uni_mapping_si2uniDyn(dxu_, x);
	
	return dxu_;

}


/* ═══ Kinematic transformations ══════════════════════════════════════════════════ */

/* create_si_to_uni_mapping: split into si2uniDyn and uni2siStates sub-functions */

MatrixXf TIGERBotUtility::create_si_to_uni_mapping_si2uniDyn(const MatrixXf& dxi,const MatrixXf& states){
yield();
    float proj_dist = 0.03; //0.05
    Eigen::Matrix2f T;
    T << 1, 0,
         0, 1/proj_dist;

    /*First mapping from SI -> unicycle.  Keeps the projected SI system at
      a fixed distance from the unicycle model
    */
    int N = dxi.cols();

    Eigen::Matrix<float,2,Dynamic> dxu(2,N); dxu.fill(0);
    Eigen::Matrix2f temp_mat;
yield();
	for (int i = 0;i< N;i++){
        temp_mat << cos(states(2,i)),sin(states(2,i)),
                    -sin(states(2,i)),cos(states(2,i));
        //cout<<"temp_mat("<<i<<"):\n"<<temp_mat<<endl;
        dxu.col(i) = T * temp_mat * dxi.col(i);
	}
	return dxu;
}

MatrixXf TIGERBotUtility::create_si_to_uni_mapping_uni2siStates(const MatrixXf& states){
yield();
    int N = states.cols();
    float proj_dist = 0.05;
	
    // Projects the single-integrator system a distance in front of the unicycle system
    Eigen::Matrix<float,2,Dynamic> xi(2,N); xi.fill(0);
    xi.row(0).array() = states.row(0).array() + proj_dist * states.row(2).array().cos();
    xi.row(1).array() = states.row(1).array() + proj_dist * states.row(2).array().sin();
//cout<<xi<<endl;
    return xi;
}
/*--------------------------------------------------*/

MatrixXf TIGERBotUtility::create_si_to_uni_mapping2(const MatrixXf& dxi, const MatrixXf& states,float lin_vel_gain, float ang_vel_lim){
/**
Returns a mapping f: R^2N * R^3N -> R^2N from single-integrator to unicycle dynamics

Detailed Description 
* LinearVelocityGain - affects the linear velocity for the unicycle
* AngularVelocityLimit - affects the upper (lower) bound for the unicycle's angular velocity

// Example Usage (Matlab)
		// si === single integrator
   si_to_uni_dynamics = create_si_to_uni_mapping2('LinearVelocityGain',1, 'AngularVelocityLimit', pi)
   dx_si = si_algorithm(si_states) 
   dx_uni = si_to_uni_dynamics(dx_si, states)
**/

yield();
    //float lin_vel_gain = 1; 		//LinearVelocityGain
    //float ang_vel_lim = PI;		//AngularVelocityLimit % default PI, use (PI/2) for DistArea4 alg.

    /* A mapping from si -> unicycle dynamics. This is more of a projection-based method.
	   Though, it's definitely similar to the NIDs. */
        int M = dxi.rows();
		int N = dxi.cols();
        int M_states = states.rows();
		int N_states = states.cols();
		
	if (M!=2)
		Serial.println("Row size of given SI velocities must be 2");
	if (M_states!=3)
		Serial.println("Row size of given poses must be 3");
    if (N!=N_states)
		Serial.println("Column sizes of SI velocities and poses must be the same");
	
	Eigen::Matrix<float,2,Dynamic> dxu(2,N); dxu.fill(0);
	Eigen::Matrix2f temp_mat;

    for (int i = 0;i< N;i++){		
        temp_mat << cos(states(2,i)),sin(states(2,i)),
                   -sin(states(2,i)),cos(states(2,i));				   
        dxu(0,i) = temp_mat.row(0)*dxi.col(i);
        dxu(0,i) = lin_vel_gain * dxu(0,i);
         // Normalizing the output of atan2 to between -kw and kw  //
		float temp_num = temp_mat.row(1)*dxi.col(i);
		float temp_den = temp_mat.row(0)*dxi.col(i);
        //float tmp = atan2(temp_mat.row(1)*dxi.col(i),temp_mat.row(0)*dxi.col(i));
		float temp = atan2(temp_num,temp_den);		
		dxu(1,i) = (ang_vel_lim/(PI/2))* temp;
	}
	
	return dxu;
}

MatrixXf TIGERBotUtility::create_si_to_uni_mapping3 (const MatrixXf& dxi, const MatrixXf& states){
								   //(const Ref<const MatrixXf>dxi, const Ref<const MatrixXf>states){                                  
                                   //(const MatrixBase<Derived>& dxi,const MatrixBase<Derived>& states){
yield();
/**
Returns a mapping f: R^2N * R^3N -> R^2N from single-integrator to unicycle dynamics

Detailed Description 
* LinearVelocityGain - affects the linear velocity for the unicycle
* AngularVelocityLimit - affects the upper (lower) bound for the unicycle's angular velocity

Example Usage 
	% si === single integrator
	si_to_uni_dynamics = create_si_to_uni_mapping2('LinearVelocityGain',1, 'AngularVelocityLimit', pi)
	dx_si = si_algorithm(si_states) 
	dx_uni = si_to_uni_dynamics(dx_si, states)
**/

    float lin_vel_gain = 1;	//LinearVelocityGain
    float ang_vel_lim = PI/4; /* AngularVelocityLimit (default PI; tune for turn sharpness) */

    /* A mapping from si -> unicycle dynamics. This is more of a projection-based method.
	   Though, it's definitely similar to the NIDs.
	*/
        int M = dxi.rows();
		int N = dxi.cols();
        int M_states = states.rows();
		int N_states = states.cols();
yield();
	if (M!=2)
		Serial.println("Row size of given SI velocities must be 2");
	if (M_states!=3)
		Serial.println("Row size of given poses must be 3");
    if (N!=N_states)
		Serial.println("Column sizes of SI velocities and poses must be the same");

	Eigen::Matrix<float,2,Dynamic> dxu(2,N); dxu.fill(0);
	Eigen::Matrix2f temp_mat;
yield();
	float angle; float s; float states2i;
	for (int i = 0;i< N;i++){
        angle = wrap(atan2(dxi(1,i), dxi(0,i)) - states(2,i));
        if(angle > -PI/2 && angle < PI/2){ // between -90 and 90 deg
            s = 1;
            temp_mat << cos(states(2,i)),sin(states(2,i)),
                       -sin(states(2,i)),cos(states(2,i));
        }
        else{
			s = -1;
            states2i = wrap(states(2,i)+PI); // Just temporarily hold value of states(2,i)
                                             // Differ from Matlab, because This function declaration doesn't
                                             // allow writing the input matrices
            temp_mat << cos(states2i),sin(states2i),
                       -sin(states2i),cos(states2i);
        }
yield();
        dxu(0,i) =(temp_mat.row(0) * dxi.col(i));
        dxu(0,i) = lin_vel_gain*dxu(0,i);
        // Normalizing the output of atan2 to between -kw and kw
        dxu(1,i) = (ang_vel_lim/(PI/2))*atan2((temp_mat.row(1) * dxi.col(i)),dxu(0,i));
        
		dxu(0,i) = s*dxu(0,i);
	}

	return dxu;
}

/* create_uni_to_si_mapping: split into uni2siDyn and si2uniStates sub-functions */

MatrixXf TIGERBotUtility::create_uni_to_si_mapping_uni2siDyn(const MatrixXf& dxu, const MatrixXf& states){
yield();
    float proj_dist = 0.03; // 0.05
    Eigen::Matrix2f T;
    T << 1, 0,
         0, proj_dist;

    /* First mapping from SI -> unicycle.  Keeps the projected SI system at
     a fixed distance from the unicycle model */
    int M = dxu.rows();
    int N = dxu.cols();
    int M_states = states.rows();
    int N_states = states.cols();

/*
    // Dimensionality checks
	if (M!=2)
		Serial.println("Row size of given SI velocities must be 2");
	if (M_states!=3)
		Serial.println("Row size of given poses must be 3");
    if (N!=N_states)
		Serial.println("Column sizes of SI velocities and poses must be the same");
*/

    Eigen::Matrix<float,2,Dynamic> dxi(2,N); dxi.fill(0);
    Eigen::Matrix2f temp_mat; temp_mat.fill(0);
    for (int i = 0;i< N;i++){
        temp_mat << cos(states(2,i)),-sin(states(2,i)),
                    sin(states(2,i)),cos(states(2,i));
        //cout<<"temp_mat("<<i<<"):\n"<<temp_mat<<endl;
        dxi.col(i) = temp_mat * T * dxu.col(i);
	}
	//cout<<"dxi:\n"<<dxi<<endl;
	return dxi;
}

MatrixXf TIGERBotUtility::create_uni_to_si_mapping_si2uniStates(const MatrixXf& uni_states, const MatrixXf& si_states){

yield();
    float proj_dist = 0.05;

        int M = uni_states.rows();
		int N = uni_states.cols();
        int M_si = si_states.rows();
		int N_si = si_states.cols();
 
    // Dimensionality checks
	if (M!=3)
		Serial.println("Row size of poses must be 3");
	if (M_si!=2)
		Serial.println("Row size of SI poses must be 2");
    if (N!=N_si)
		Serial.println("Column size of poses must be the same as SI poses");
yield();
    /* Projects the single-integrator system a distance in front of the
       unicycle system */
    Eigen::Matrix<float,2,Dynamic> xi(2,N); xi.fill(0);
    xi.row(0).array() = si_states.row(0).array() - proj_dist * uni_states.row(2).array().cos();
    xi.row(1).array() = si_states.row(1).array() - proj_dist * uni_states.row(2).array().sin();

    return xi;
}
/*---------------------------------------------------------------------------*/



/* ═══ Controller functions ═══════════════════════════════════════════════════════ */
MatrixXf TIGERBotUtility::create_automatic_parking_controller2(const MatrixXf& states, const MatrixXf& poses,float pos_err,float rot_err){
/**
create_automatic_parking_controller2
Returns a controller (u:R^{3N} * R^{3N} -> R^{2N}) that automatically parks agents 
at desired poses, zeroing out their velocities when the point (within a tolerance) is
reached.

// Detailed Description
This function returns a controller that allows for agents to be parked at a desired 
position and orientation.  When the agents are within the error bounds, 
this function will automatically stop their movement.
 
* ApproachAngleGain - affects how the unicycle approaches the desired position
* DesiredAngleGain - affects how the unicycle approaches th desired angle
* RotatationErrorGain - affects how quickly the unicycle corrects rotation errors

// Example Usage (MATLAB)
parking_controller = CREATE_AUTOMATIC_PARKING_CONTROLLER2('ApproachAngleGain', 1,'DesiredAngleGain', 1, 'RotationErrorGain', 1)
**/
yield();
    float lin_vel_gain = 1;   		// LinearVelocityGain, orig: .75
    float ang_vel_limit = PI/4; 	// AngularVelocityLimit, orig: pi/2
    float vel_mag_limit = 0.035;  	// VelocityMagnitudeLimit, orig: .08
    /* TO DO: expose as default arguments with safe fallback values */
	//float pos_err = 0.01;			// PositionError
    //float rot_err = 0.25;			// RotationError

    int N = states.cols();
	Eigen::Matrix<float,2,Dynamic> dxu(2,N); dxu.fill(0);
	Eigen::Vector2f dxi; dxi.fill(0);

    float wrapped; float norm_;
    for (int i = 0;i<N; i++){
        wrapped = poses(2,i) - states(2,i);
        wrapped = atan2(sin(wrapped),cos(wrapped));
yield();
        // Calculates distance(x,y)(coord.) between current & target(or initial) position
        dxi = poses.col(i).head<2>() - states.col(i).head<2>(); // array operation..

        // Normalize
        norm_ = dxi.norm(); // norm of the distance[2x1] matrix ((x,y) coordinate)
        if(norm_ > vel_mag_limit)
            dxi = (vel_mag_limit/norm_)*dxi;
        else
            dxi=dxi;

        if(dxi.norm() > pos_err)
            dxu.col(i) = create_si_to_uni_mapping3(dxi,states.col(i)); // position controller in Matlab			            
		else if(abs(wrapped) > rot_err){
            dxu(0,i) = 0;
            dxu(1,i) = wrapped;
        }
        else 
            dxu.col(i).fill(0);
        
    }
	return dxu;
}

/* ═══ Initial conditions ═════════════════════════════════════════════════════════ */

MatrixXf TIGERBotUtility::getPolygonDims(float radius, int n, int desired_output) {
/* desired_output: 2->chords, 3->P2 (goal poses), 4->P3 (randomized initial conditions) */

/* configure polygon shape: circleType=1 for circumscribed, circleType=2 for inscribed */
yield();	
	int circleType = 1;	// 2 for INSCRIBED 
						// 1 for CIRCUMSCRIBED

	//int n=10;                 // define number of sides
	// L = 0.2;                  // define side length of polygon

	RowVector3f transform; // define shift x,y and CCW rotation in degrees
	transform << 0.35, 0.35, -135;

	Eigen::Matrix<float,1,2> X ; X <<0,0.08;
	Eigen::Matrix<float,1,2> T ; T <<0,2*PI;

	/* RANGE OF X & T */
	float range_X = X.maxCoeff() - X.minCoeff(); //cout<<"\nrange X:"<<range_X<<endl;
	float range_T = T.maxCoeff() - T.minCoeff(); //cout<<"\nrange T:"<<range_T<<endl;
yield();
	//------------------------------------
	/* Creating a warp function...*/
	srand(unsigned(time(NULL)));
	//Add in a shear transform to create initial conditions
	Eigen::Matrix<float,Dynamic,3> warp_(n,3);
	Eigen::Matrix<float,3,Dynamic> warp(3,n);

	for(int rw=0;rw<n;rw++){
		for(int col=0;col<3;col++){
		            /* generate uniform random offset in [-range_X, 0] for scattered initial positions */
			double randNum = -((double)rand()/(RAND_MAX + 1)+0+(rand()%1));
			//cout<<"\nrandom Number:"<<randNum<<endl;
			if(col<2) 
				warp_(rw,col)= randNum * range_X + X.minCoeff();
			else       
				warp_(rw,col)= randNum * range_T + T.minCoeff();
		}
	}
	// TRANSPOSE OF warp_[n][3]
	warp = warp_.transpose();
	cout<<"\nwarp:\n"<<warp<<endl;
	//-----------------------------------

	float d = 360/n;  // degrees between corners   //cout<<"\nd:"<<d<<endl;
yield();
	// relationship coefficient between side length and circle radius
	Eigen::Matrix<float,2,10> radii;
	radii << 0,0,0.28867,0.5,0.68819,0.86602,1.0383,1.2071,1.3737,1.5388,
		     0,0,0.57735,0.70710,0.85065,1,1.1523,1.3065,1.4619,1.6180;
		 
	// radius = radii(circleType, n-1) * L
	float L = radius/radii(circleType,n-1);  // OUTPUT: side length ( -1 because of index change in C++)
//cout<<"\nradii:"<<radii(circleType,n-1)<<endl;
//cout<<"L:"<<L<<endl;
	//--------------------------------------
	Eigen::Matrix<float,2,Dynamic> P1(2,n); // initialize coordinates for each vertex of polygon
	P1.setZero(2,n);
	

	// *** *** ERROR IN MATLAB code ( case swapped) *** ***
	circleType = 2; // *** *** ERROR IN MATLAB code ( case swapped) *** ***
	// *** *** ERROR IN MATLAB code ( case swapped) *** ***	
	
	
	/* calculate coordinates of each vertex with center on the origin*/
	switch (circleType){
	case 1:// polygon is CIRCUMSCRIBED around the circle
	// SET P1 MATRIX
	for(int rw=0; rw<2; rw++){
		for (int col=0; col<n; col++){
			// point 1 and then assigned in CCW direction
			if(rw==0)
				P1(rw,col) = (radius/(cos((d/2)*M_PI/180)))*(cos((col+1)*d*M_PI/180));
			else
				P1(rw,col) = (radius/(cos((d/2)*M_PI/180)))*(sin((col+1)*d*M_PI/180));
		}
	}
	//SET TOLERANCE FOR P1 MATRIX ELEMENTS : Make it 0 if smaller than tolerance value
	for(int rw=0; rw<2; rw++){
		for (int col=0; col<n; col++){
            if(P1(rw,col)<1e-1 && P1(rw,col)>-(1e-1))
                    P1(rw,col)=0;
		}
	}

	break;

	case 2:// polygon is INSCRIBED in the circle
	// SET P1 MATRIX
	for(int rw=0; rw<2; rw++){
		for (int col=0; col<n; col++){
			// point 1 and then assigned in CCW direction
			if(rw==0)
                P1(rw,col) = radius*(cos((col+1)*d*M_PI/180));
			else
			    P1(rw,col) = radius*(sin((col+1)*d*M_PI/180));
		}
	}
	//SET TOLERANCE FOR P1 MATRIX ELEMENTS: Make it 0 if smaller than tolerance value
	for(int rw=0; rw<2; rw++){
		for (int col=0; col<n; col++){
            if(P1(rw,col)<1e-1 && P1(rw,col)>-(1e-1))
                    P1(rw,col)=0;
		}
	}
	break;
	}
	//cout<<"\nP1:\n"<<P1<<endl;
	//--------------------------------------------
yield();
	Eigen::Matrix<float,1,Dynamic> chords(1,n-1); chords.setZero(1,n-1);
	for(int i=1;i<n;i++)
		chords(0,i-1)=(P1.col(i)-P1.col(0)).norm();

	/* Carry out transformation calculations, if desired. This will translate
     and rotate the polygon based on the values provided in transform (3x1). */
	Eigen::Matrix<float,2,2> R; 
	R<< cos(transform(2)*M_PI/180),-sin(transform(2)*M_PI/180),
		sin(transform(2)*M_PI/180),cos(transform(2)*M_PI/180);
           
   //cout<<"R_in:\n"<<R<<endl;

	Eigen::Matrix<float,2,Dynamic> P2(2,n); P2.setZero(2,n);
	// SET P2 MATRIX
    P2 = R*P1;
    
	for(int i=0;i<n;i++)
            P2.col(i)= P2.col(i)+(transform.head<2>()).transpose();
    cout<<"\nP2:\n"<<P2;
		
	//----------------------------------------------
	/* Warp/scatter the polygon shape in a randomized fashion. This can be used
     to generate random initial conditions. */
	Eigen::Matrix<float,3,Dynamic> P3(3,n); P3.setZero(3,n);
	P3.topRows(2)=P3.topRows(2)+ P2;
	P3 = P3 + warp;
	//cout<<"\nP3:\n"<<P3;

	//if (desired_output ==1)		return L; // fix this. L can't be converted as MatrixXf
	if (desired_output ==2)		return chords;
	if (desired_output ==3)		return P2;
	if (desired_output ==4)		return P3;
	
}

/* ═══ Helper functions ═══════════════════════════════════════════════════════════ */
// Calculate N choose k
int TIGERBotUtility::factorial(int num){
	int fact=1;
	for(int i=1;i<=num;i++)
		fact = fact*i;
return fact;
}

int TIGERBotUtility::nchoosek(int n, int k){
	int nCk = factorial(n)/(factorial((n-k))*(factorial(k)));
return nCk;
}

float TIGERBotUtility::wrap(float theta){
    float result = atan2(sin(theta),cos(theta));
	return result;
}

/* ═══ Parameter setter stubs ═════════════════════════════════════════════════════ */
/* TO DO: parameter setter methods -- not yet implemented
    float setSafetyRadius(float safety){
		safety_radius = safety;
		
		return safety_radius;
	}
	float setBarrierGain(){
		barrier_gain =	
		
		return barrier_gain;
	}
	
	float setPositionError(float PositionError){
		pos_err = PositionError;
		
		return pos_err;
	}
	float setRotationError(float RotationError){
		rot_err = RotationError;
		
		return rot_err;
	}
		
	float setProjectionDistance(float lambda){
		proj_dist = 
		
		return proj_dist;
	}
		
	float setVelocityMagnitudeLimit(){
		vel_mag_lim =
		
		return vel_mag_lim;
	}
		
	float setLinearVelocityGain(float linearVelocityGain){
		lin_vel_gain = linearVelocityGain;
		
		return lin_vel_gain;
	}
	float setAngularVelocityGain(float angularVelocityGain){
		ang_vel_lim = angularVelocityGain;
		
		return ang_vel_lim;
	}


*/


