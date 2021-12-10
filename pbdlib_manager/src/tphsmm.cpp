/*
 * tphsmm.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Ajay Tanwani
 */

#include "pbdlib/tphsmm.h"
namespace pbdlib
{
TPHSMM::TPHSMM(uint _nVARS, uint _nSTATES, uint _nTPs): HSMM(_nSTATES, _nVARS){
	this->nbVARPOS = _nVARS;
	this->nSTATES = _nSTATES;
	tpgmm = new TPGMM(_nVARS, _nSTATES, _nTPs);
	taskparam = new TaskParameters(_nVARS, _nTPs);
	double dt = 0.1f;
	double rf = 1E-6;
	init_TPHSMM_LQR(dt, rf);
}

TPHSMM::TPHSMM(uint _nVARS, uint _nSTATES, uint _nTPs, uint _nbVARPOS, double dt, double rf): HSMM(_nSTATES, _nVARS){
	this->nbVARPOS = _nbVARPOS;
	this->nSTATES = _nSTATES;
	tpgmm = new TPGMM(_nVARS, _nSTATES, _nTPs);
	taskparam = new TaskParameters(_nVARS, _nTPs);
	init_TPHSMM_LQR(dt, rf);
}

TPHSMM::TPHSMM(uint _nVARS,uint _nTPS, float _minSigma): HSMM(1,_nVARS){

	this->nTPS = _nTPS;
    this->minSigma = _minSigma;
    TPTRANSITION transTmp(_nTPS);
    std::vector<TPTRANSITION> outTransTmp;
    outTransTmp.push_back(transTmp);

    TPTransitionMatrix.push_back(outTransTmp);
	hsmm_priors_ticks = zeros(1);
    colvec MuDurTmp = zeros(1);
    mat SigmaDurTmp = eye(1,1);
    GaussianDistribution DurTmp(MuDurTmp,SigmaDurTmp);
    DurationCOMPONENTS.push_back(DurTmp);
    this->nSTATES = 1;
}

TPHSMM::TPHSMM(std::vector<GMM_Model> GMMS, mat transition, std::vector<GaussianDistribution> components, uint _nVARS, uint _nSTATES, uint _nTPs): HSMM(_nSTATES, _nVARS){

//	this->nTaskParameters = this->GMMS.size();
	tpgmm = new TPGMM(GMMS);

	this->nSTATES = GMMS[0].getNumSTATES();
	this->nbVARPOS = GMMS[0].getNumVARS();

	taskparam = new TaskParameters(_nVARS, _nTPs);

	TransitionMatrix = transition;

	// Set state duration components:
	mat _SIGMA = zeros(1,1);
	colvec _MU = zeros(1,1);
//	std::vector<GaussianDistribution> components;
//
//	for(uint i=0; i<_nSTATES; i++){
//		_MU(0,0) = durMu(0,i);
//		_SIGMA(0,0) = durSigma(0,i);
//		// for debugging
//		components.push_back(GaussianDistribution(_MU, _SIGMA));
//	}
	setDurationCOMPONENTS(components);

	// Forward variable calculation initialization:
	initializeFwdCalculation();

	double dt = 0.1f;
	double rf = 1E-6;
	init_TPHSMM_LQR(dt, rf);
}

TPHSMM::TPHSMM(std::string path,uint _nSTATES,uint _nVARS,uint _nTPS, float _minSigma): HSMM(1,_nVARS){

	this->nTPS = _nTPS;
	this->minSigma = _minSigma;
	this->nSTATES = _nSTATES;
	mat mu,nTrans;
	cube sigma;

	nTrans.load(path + "nTrans_tp_transition.txt", raw_ascii);
	mu.load(path + "mu_tp_transition.txt", raw_ascii);
	sigma.load(path + "sigma_tp_transition.txt", arma_binary);

	for(int i=0;i<nSTATES;i++){
		std::vector<TPTRANSITION> outTransTmp;
		for (int j = 0; j < nSTATES; j++){
			TPTRANSITION transTmp(_nTPS);

			colvec MuTmp(_nTPS);
			mat SigmaTmp(_nTPS,_nTPS);

			for(int i_T=0;i_T<nTPS;i_T++){
				MuTmp(i_T)= mu(j + i * nSTATES, i_T);
				for (int j_T = 0; j_T < nTPS; j_T++)
				{
					SigmaTmp(i_T,j_T) = sigma(j + i * nSTATES, i_T, j_T);
				}
			}
			transTmp.transParam->setMU(MuTmp);
			transTmp.transParam->setSIGMA(SigmaTmp);
			transTmp.nTrans = nTrans(i,j);
			outTransTmp.push_back(transTmp);
		}
		TPTransitionMatrix.push_back(outTransTmp);
	}
	hsmm_priors_ticks.load(path + "hsmm_priors_ticks.txt", raw_ascii);

	mat durMu, durSigma;

	// Load state duration components
	durMu.load(path + "durMu.txt", raw_ascii);
	durSigma.load(path + "durSigma.txt", raw_ascii);

	// Set state duration components:
	mat _SIGMA = zeros(1,1);
	colvec _MU = zeros(1,1);
	std::vector<GaussianDistribution> components;

	for(uint i=0; i<_nSTATES; i++){
		_MU(0,0) = durMu(0,i);
		_SIGMA(0,0) = durSigma(0,i);
		// for debugging
		components.push_back(GaussianDistribution(_MU, _SIGMA));
	}
	setDurationCOMPONENTS(components);

	// Forward variable calculation initialization:
	initializeFwdCalculation();
}

void TPHSMM::init_TPHSMM_LQR(double dt, double rf){
	InitialState.zeros(tpgmm->getNumVARS(),1);
	CurrentState = InitialState;

	mat A, B;

	mat A1d; A1d << 0 << 1 << endr << 0 << 0 << endr;
	mat B1d; B1d << 0 << endr << 1 << endr;

	A = kron(A1d, eye(nbVARPOS,nbVARPOS));
	B = kron(B1d, eye(nbVARPOS,nbVARPOS));

	lqr = new LQR(A,B,dt);
	pred_length = 200;
	RegularizationConstLQR = 1E-3;
	HorizonMethod = 0;
	SamplingMethod = 0;
	SimMethod = 0;

	mat R = eye(nbVARPOS,nbVARPOS) * rf;
	lqr->setR(R);
}

/*
 * variable knobs to keep
 * 		- diagRegularziation Mat
 * 		- R
 * 		- pred_length
 * 		- infinite horizon vs finite horizon
 * 		- stochastic sampling vs deterministic sampling
 */


//TPHSMM::TPHSMM(std::string PriorsFileName,
//				std::string VarsFileName,
//				std::string MuFilePrefix,
//				std::string SigmaFilePrefix,
//				const std::string &transition_path,
//				const std::string &durMu_path,
//				const std::string &durSigma_path){
//
//	// load TPGMM
//	//loadTPHSMMfromFiles(PriorsFileName,VarsFileName,MuFilePrefix, SigmaFilePrefix);
//	loadTPGMMfromMATLAB(PriorsFileName,VarsFileName,MuFilePrefix, SigmaFilePrefix);
//
//	// load transition matrix
//	mat transition;
//	transition.load(transition_path,raw_ascii);
//	TransitionMatrix = transition;
//
//	// load duration probability distributions
//	mat durMu, durSigma;
//
//	// Load state duration components
//	durMu.load(durMu_path, raw_ascii);
//	durSigma.load(durSigma_path, raw_ascii);
//
//	// Set state duration components:
//	mat _SIGMA = zeros(1,1);
//	colvec _MU = zeros(1,1);
//	std::vector<GaussianDistribution> components;
//
//	for(uint i=0; i<this->nSTATES; i++){
//		_MU(0,0) = durMu(0,i);
//		_SIGMA(0,0) = durSigma(0,i);
//		// for debugging
//		components.push_back(GaussianDistribution(_MU, _SIGMA));
//	}
//	setDurationCOMPONENTS(components);
//
//	// Forward variable calculation initialization:
//	initializeFwdCalculation();
//
//}

TPHSMM::~TPHSMM() {
	// TODO Auto-generated destructor stub
	delete lqr;
	delete tpgmm;
	delete taskparam;
}


void TPHSMM::setSamplingMethod(uint SamplingMethod){
	//SamplingMethod = 0 implies deterministic sampling
	//SamplingMethod = 1 implies stochastic sampling
	assert(SamplingMethod == 0 || SamplingMethod == 1);
	this->SamplingMethod = SamplingMethod;
}

void TPHSMM::setLQRHorizonMethod(uint HorizonMethod){
	//HorizonMethod = 0 implies infinite Horizon
	//HorizonMethod = 1 implies finite Horizon
	assert(HorizonMethod == 0 || HorizonMethod == 1);
	this->HorizonMethod = HorizonMethod;
}

void TPHSMM::setSimulationMethod(uint SimMethod){
	assert(SimMethod == 0 || SimMethod == 1);
	this->SimMethod = SimMethod;
}

void TPHSMM::setPredictionLength(uint PredictionLength){
	this->pred_length = PredictionLength;
}

void TPHSMM::setScalingR(double rf){

//	mat R = eye(nbVARPOS,nbVARPOS) * rf;
	mat R = eye(nbVARPOS,nbVARPOS) * pow(10.0f,rf);
	lqr->setR(R);
}

void TPHSMM::setRegularizationConstLQR(double RC){
//	mat R = eye(nbVARPOS,nbVARPOS) * pow(10.0f,rf);
	RegularizationConstLQR = pow(10.0f,RC);
}

void TPHSMM::loadTPHSMMfromFiles(std::string PriorsFileName,
				std::string VarsFileName,
				std::string MuFilePrefix,
				std::string SigmaFilePrefix,
				const std::string &transition_path,
				const std::string &durMu_path,
				const std::string &durSigma_path){

	tpgmm->loadTPGMMfromMATLAB(PriorsFileName,VarsFileName,MuFilePrefix, SigmaFilePrefix);

	// load transition matrix
	mat transition;
	transition.load(transition_path,raw_ascii);
	TransitionMatrix = transition;

	// load duration probability distributions
	mat durMu, durSigma;

	// Load state duration components
	durMu.load(durMu_path, raw_ascii);
	durSigma.load(durSigma_path, raw_ascii);

	// Set state duration components:
	mat _SIGMA = zeros(1,1);
	colvec _MU = zeros(1,1);
	std::vector<GaussianDistribution> components;

	for(uint i=0; i<this->getNumSTATES(); i++){
		_MU(0,0) = durMu(0,i);
		_SIGMA(0,0) = durSigma(0,i);
		// for debugging
		components.push_back(GaussianDistribution(_MU, _SIGMA));
	}
	setDurationCOMPONENTS(components);

	// Forward variable calculation initialization:
	initializeFwdCalculation();

}

void TPHSMM::setCurrentTaskFrameFromFile(std::string FramePath){
	taskparam->loadFromFile(FramePath);
}

void TPHSMM::setCurrentTaskFrame(std::vector<TaskParameter> TaskParameters){
	taskparam->setTaskParameters(TaskParameters);
}

std::vector<TaskParameter>& TPHSMM::getTaskParameters(){
	return taskparam->getTaskParameters();
}

mat TPHSMM::HSMMLQR(GMM_Model& GMM, colvec init_state, uint pred_length){

	for (int i=0; i < tpgmm->getNumVARS(); i++){
		cout << init_state[i] << endl;
	}
	mat h_i(this->getNumSTATES(), 1);

	for (int i=0; i<this->getNumSTATES(); i++){
		h_i.row(i) = trans(GMM.getCOMPONENTS(i).getPDFValue(init_state));
	}
	uword start_state;
	h_i.max(start_state);

	//Predict the forward variable
	AlphaHSMM.zeros(this->getNumSTATES(), pred_length);

	if (SamplingMethod == 0){
		predictForwardVariableDeterministic(AlphaHSMM, start_state);
	}else{
		predictForwardVariableStochasticStart(AlphaHSMM, start_state);
	}

	uword StateSequence[pred_length];

	get_state_seq(StateSequence, AlphaHSMM);

	//Set Q and Target
	std::vector<mat> Qt;
	mat Q(nbVARPOS*2,nbVARPOS*2, fill::zeros);
	mat S_T(nbVARPOS*2,nbVARPOS*2, fill::zeros);
	colvec d_T(nbVARPOS*2, fill::zeros);
	mat Target(nbVARPOS*2, pred_length, fill::zeros);
	vec vTmp(nbVARPOS*2, fill::zeros);
	mat RegularizationMat = eye(tpgmm->getNumVARS(),tpgmm->getNumVARS())*RegularizationConstLQR;

	int counter = 0;
	for (int i=0; i<pred_length-1; i++){
		if (StateSequence[i] == StateSequence[i+1]){
			counter = counter + 1;
		}
		else{
			cout << "State " << i << ",\t" << StateSequence[i] << ",\t" << counter << endl;
			counter = 0;
		}
	}
	cout << "State " << pred_length-1 << ",\t" << StateSequence[pred_length-1] << ",\t" << counter << endl;


	for (int i=0; i < pred_length; i++){
//		cout << i << ",\t" << StateSequence[i] << endl;
		Q.submat(0,0,tpgmm->getNumVARS()-1,tpgmm->getNumVARS()-1) = inv(ProdGauss->getSIGMA(StateSequence[i]) +  RegularizationMat);
		Qt.push_back(Q);
		vTmp.subvec(0,tpgmm->getNumVARS()-1) = ProdGauss->getMU(StateSequence[i]);
		Target.col(i) = vTmp;
	}

	// set up the lqr problem
	lqr->setProblem(lqr->getR(), Qt, Target);
	if (HorizonMethod == 1){
		lqr->evaluate_gains_finiteHorizon(S_T, d_T);
	}else{
		lqr->evaluate_gains_infiniteHorizon();
	}


//	S_T = lqr->solveAlgebraicRiccati(lqr->getA(), lqr->getB(), (Qt[pred_length-1] + Qt[pred_length-1].t())/2, lqr->getR());

	std::vector<mat> S = lqr->getS();
	std::vector<mat> L = lqr->getGains();
//	for (int k = pred_length-1; k >= 0; k--){
//		cout << k << endl;
//		for (int i=0; i< nbVARPOS; i++){
//			for (int j=0; j < TPGMM::getNumVARS(); j++){
//	//			cout << S_T.at(i,j) << ",\t";
//	//			cout << Qt[pred_length - 1].at(i,j) << ",\t";
//				cout << L[k].at(i,j) << ",\t";
//			}
//			cout << endl;
//		}
//		cout << endl << endl;
//	}
//
//	cout << "###############################################" << endl << endl;
//	for (int k = pred_length-1; k >= 0; k--){
//		cout << k << endl;
//		for (int i=0; i< S_T.n_rows; i++){
//			for (int j=0; j < S_T.n_cols; j++){
//	//			cout << S_T.at(i,j) << ",\t";
//	//			cout << Qt[pred_length - 1].at(i,j) << ",\t";
//				cout << S[k].at(i,j) << ",\t";
//			}
//			cout << endl;
//		}
//		cout << endl << endl;
//	}
//	cout << "dt: " << lqr->getdt() << endl;
	//Retrieve data
	mat rD(nbVARPOS, pred_length, fill::zeros);

	if (SimMethod == 0){

		vec x = init_state.subvec(0,nbVARPOS-1);
		vec dx(nbVARPOS, fill::zeros);    // [x dx] = join_cols(init_state, zeros<vec>(nbVARPOS));
		vec u(nbVARPOS);

		for (int t=0; t<pred_length; t++){
			u = - lqr->getGains().at(t)*(join_cols(x, dx) - Target.col(t)) + lqr->getFF().at(t);
	//		u = - lqr->getGains().at(t) * (x - Target.col(t));
			dx += u*lqr->getdt();
			x += dx*lqr->getdt();
			setCurrentState(x);
			rD.col(t) = x;
		}
	}else{
		vec u(nbVARPOS);
		vec X(nbVARPOS*2,fill::zeros);
		X.subvec(0,nbVARPOS-1) = init_state.subvec(0,nbVARPOS-1); //startPoint.rows(0, tpgmm.getNumVARS()-1 );

		for (int t=0; t < pred_length; t++){
			rD.col(t) = X.rows(0,nbVARPOS-1);
			u = lqr->getGains().at(t) * (Target.col(t)-X);
			X += (lqr->getA()*X+lqr->getB()*u) * lqr->getdt();
		}
	}
	return rD;
}

mat TPHSMM::TPHSMMController(std::vector<TaskParameter> cTaskParam, colvec init_state, uint pred_length){

	//Take the product of Gaussians
	ProdGauss = tpgmm->getTransformedGMM(cTaskParam, OTHER);
	//printGMM(*ProdGauss);
	setInitialState(init_state);
	rData = HSMMLQR(*ProdGauss, getInitialState(), pred_length);
	return rData;

}

mat TPHSMM::TPHSMMController(std::vector<TaskParameter> cTaskParam, uint pred_length){
	//Take the product of Gaussians
	ProdGauss = tpgmm->getTransformedGMM(cTaskParam, OTHER);
	setInitialState(*ProdGauss);

	rData = HSMMLQR(*ProdGauss, getInitialState(), pred_length);
	return rData;
}

mat TPHSMM::TPHSMMController(){
	cTaskParam = taskparam->getTaskParameters();

	//Take the product of Gaussians
	ProdGauss = tpgmm->getTransformedGMM(cTaskParam, OTHER);
	setInitialState(*ProdGauss);

	rData = HSMMLQR(*ProdGauss, getInitialState(), pred_length);
	return rData;
}

void TPHSMM::setCurrentState(colvec state){
	CurrentState = state;
}

void TPHSMM::setInitialState(colvec init_state){
	InitialState = init_state;
}

void TPHSMM::setInitialState(GMM_Model& ProdGauss){
	InitialState.zeros(tpgmm->getNumVARS(),1);
	InitialState.subvec(0, tpgmm->getNumVARS() - 1) = ProdGauss.getMU(0);
}

colvec& TPHSMM::getCurrentState(){
	return CurrentState;
}

colvec& TPHSMM::getInitialState(){
	return InitialState;
}


TPGMM* TPHSMM::get_TPGMM(){
	return this->tpgmm;
}

GMM_Model* TPHSMM::get_ProdGMM(std::vector<TaskParameter> cTaskParam){
	ProdGauss = tpgmm->getTransformedGMM(cTaskParam, OTHER);
	return ProdGauss;
}

LQR* TPHSMM::get_LQR(){
	return this->lqr;
}

//uint TPHSMM::getNumSTATES(){
//	return nSTATES;
//}

void TPHSMM::printGMM(GMM_Model& GMM){

	for (int i=0; i<tpgmm->getNumSTATES(); i++){
		std::cout << "Gaussian Component id: " << i << std::endl;
		std::cout << "Prior: " << GMM.getPRIORS(i) << std::endl;
		printGaussian(GMM.getCOMPONENTS(i));
	}
}

void TPHSMM::printGaussian(GaussianDistribution& GaussPr){

	colvec MuTmp = GaussPr.getMU();
	mat SigmaTmp = GaussPr.getSIGMA();

	std::cout << "Mean: " << std::endl;
	for (int i=0; i < tpgmm->getNumVARS(); i++){
		std::cout << MuTmp[i] << ",\t";
	}
	std::cout << std::endl;

	std::cout << "Covariance Matrix: " << std::endl;
	for (int i=0; i < tpgmm->getNumVARS(); i++){
		for (int j=0; j < tpgmm->getNumVARS(); j++){
			std::cout << SigmaTmp.at(i,j) << ",\t";
		}
		std::cout << std::endl;
	}
}

void TPHSMM::updateTransitionMat(vec _trParam)
{
    TransitionMatrix  = zeros(nSTATES,nSTATES);
    TransCount = zeros(nSTATES,nSTATES);

    for(int i =0;i<nSTATES;i++) {
        for(int j=0;j<nSTATES;j++){
            mat param = ones(nTPS,1);
            // evaluate transition probability using current parameters
            TransCount(i,j) = TPTransitionMatrix[i][j].nTrans;
			TransitionMatrix(i,j) = TPTransitionMatrix[i][j].nTrans *
                                    (TPTransitionMatrix[i][j].transParam->getPDFValue(_trParam))(0);
        }
        double sumTmp = sum(TransitionMatrix.row(i));
        //if (sumTmp>std::numeric_limits<double>::epsilon())
            TransitionMatrix.row(i) = TransitionMatrix.row(i)/sumTmp;
    }

}

void TPHSMM::addTransition(uint _prevSTATE, uint _newSTATE, vec _trParam,bool startFlag, uint _timeStepCNT)
{
    // ******  new state and new transition
    if(_newSTATE>=nSTATES) {
        std::vector<TPTRANSITION> outTransTmp;
        for (int i = 0; i < nSTATES; i++)// for each existing states create a transition to the new
        {
            if (i == _prevSTATE&&!startFlag) { // create an existing transition with this state
                TPTRANSITION transTmp(nTPS, _trParam, minSigma);
                TPTransitionMatrix[i].push_back(transTmp);

            }
            else { // create an empty transition for the other
                TPTRANSITION transTmp(nTPS);
                TPTransitionMatrix[i].push_back(transTmp);
            }
            // create a transition from new state to all the other
            TPTRANSITION transTmp(nTPS);
            outTransTmp.push_back(transTmp);
        }
        // create empty self-transition for new state
        TPTRANSITION transTmp(nTPS);
        outTransTmp.push_back(transTmp);

        // add transition going out of new state to transition matrix
        TPTransitionMatrix.push_back(outTransTmp);

        colvec MuDurTmp = zeros(1);
        mat SigmaDurTmp = eye(1,1);
        GaussianDistribution DurTmp(MuDurTmp,SigmaDurTmp);
        DurationCOMPONENTS.push_back(DurTmp);
        //nbStatesVisit.push_back(0);
		rowvec hsmm_priors_ticksTmp = zeros(1,nSTATES+1);
		if (nSTATES>1)
			hsmm_priors_ticksTmp.cols(0,nSTATES-1) = hsmm_priors_ticks;

		hsmm_priors_ticks = hsmm_priors_ticksTmp;

        nSTATES++;


    }
    else if(!startFlag) {// update transition between existing states
        TPTransitionMatrix[_prevSTATE][_newSTATE].addTransition(_trParam,minSigma);
    }
    if(!startFlag)
        updateDuration(_prevSTATE,_timeStepCNT);

    //for(int i =0;i<nSTATES;i++)
      //  cout<<nbStatesVisit[i];
    //cout<<endl;

}
void TPHSMM::updateDuration(uint _idxSTATE, uint _timeStepCNT) {
//        nbStatesVisit[_idxSTATE]++;
	hsmm_priors_ticks(_idxSTATE) = hsmm_priors_ticks(_idxSTATE) +1;
//		for(int i=0;i<nSTATES;i++)
//			cout<<nbStatesVisit[i];
//		cout<<endl;
//		cout<<hsmm_priors_ticks<<endl;
    uint N = hsmm_priors_ticks(_idxSTATE);

    colvec MuTmp = 1.0/N * ((N-1.0)*DurationCOMPONENTS[_idxSTATE].getMU() + _timeStepCNT);
    mat SigmaTmp;
    if(N>1){
        SigmaTmp = 1.0/N * ((N-1.0)*DurationCOMPONENTS[_idxSTATE].getSIGMA() +
                (double)(N/(N-1.0))*((_timeStepCNT-MuTmp)*(_timeStepCNT-MuTmp).t()));
    }
    else SigmaTmp = MuTmp(0)*eye(1,1);
    DurationCOMPONENTS[_idxSTATE].setMU(MuTmp);
    DurationCOMPONENTS[_idxSTATE].setSIGMA(SigmaTmp);
}

double TPHSMM::getTransSimilarity(uint _k, uint _l)
{
//	uint nSTATES = TPGMM::getNumSTATES();
	for (uint i=0;i<nSTATES;i++)
	{
		if(TPTransitionMatrix[_k][i].nTrans*TPTransitionMatrix[_l][i].nTrans!=0)
			return 1.0;
	}
	return 0.0;
}
void TPHSMM::mergeTransition(uint _k, uint _l)
{
//	uint nSTATES = TPGMM::getNumSTATES();
	for (int i = 0; i < nSTATES; i++)
	{
		if(i==_k||i==_l)
			continue;
		// for each state merge TO deleted state
		uint nTransTmp = TPTransitionMatrix[i][_l].nTrans + TPTransitionMatrix[i][_k].nTrans;
		if(nTransTmp>0)
		{
			colvec MuTmp = 1.0/nTransTmp *
					(TPTransitionMatrix[i][_l].nTrans *TPTransitionMatrix[i][_l].transParam->getMU() +
					 TPTransitionMatrix[i][_k].nTrans *TPTransitionMatrix[i][_k].transParam->getMU());

			mat SigmaTmp = 1.0/nTransTmp *
				(TPTransitionMatrix[i][_l].nTrans * (TPTransitionMatrix[i][_l].transParam->getSIGMA() +
					(TPTransitionMatrix[i][_l].transParam->getMU()-MuTmp) *
							(TPTransitionMatrix[i][_l].transParam->getMU()).t())+
				TPTransitionMatrix[i][_k].nTrans * (TPTransitionMatrix[i][_k].transParam->getSIGMA() +
					(TPTransitionMatrix[i][_k].transParam->getMU()-MuTmp) *
							(TPTransitionMatrix[i][_k].transParam->getMU()).t()));

			TPTransitionMatrix[i][_l].transParam->setMU(MuTmp);
			TPTransitionMatrix[i][_l].transParam->setSIGMA(SigmaTmp);
			TPTransitionMatrix[i][_l].nTrans = nTransTmp;
		}
		// for each state merge from deleted state
		nTransTmp = TPTransitionMatrix[_l][i].nTrans + TPTransitionMatrix[_k][i].nTrans;

		if(nTransTmp>0)
		{
			colvec MuTmp = 1.0 / nTransTmp *
					(TPTransitionMatrix[_l][i].nTrans * TPTransitionMatrix[_l][i].transParam->getMU() +
					 TPTransitionMatrix[_k][i].nTrans * TPTransitionMatrix[_k][i].transParam->getMU());

			mat SigmaTmp = 1.0 / nTransTmp *
					   (TPTransitionMatrix[_l][i].nTrans * (TPTransitionMatrix[_l][i].transParam->getSIGMA() +
							   (TPTransitionMatrix[_l][i].transParam->getMU() - MuTmp) *
						   			(TPTransitionMatrix[_l][i].transParam->getMU()).t()) +
					   TPTransitionMatrix[_k][i].nTrans * (TPTransitionMatrix[_k][i].transParam->getSIGMA() +
							   (TPTransitionMatrix[_k][i].transParam->getMU() - MuTmp) *
									(TPTransitionMatrix[_k][i].transParam->getMU()).t()));

			TPTransitionMatrix[_l][i].transParam->setMU(MuTmp);
			TPTransitionMatrix[_l][i].transParam->setSIGMA(SigmaTmp);
			TPTransitionMatrix[_l][i].nTrans = nTransTmp;
		}
	}
	for (int i = 0; i < nSTATES; i++)// for each state merge and remove transition TO deleted state
	{
		TPTransitionMatrix[i].erase(TPTransitionMatrix[i].begin()+_k);
	}
	//remove all transition FROM deleted state
	//nbStatesVisit.erase(nbStatesVisit.begin()+_k);
	hsmm_priors_ticks.shed_col(_k);
	TPTransitionMatrix.erase(TPTransitionMatrix.begin()+_k);
	nSTATES--;

}

void TPHSMM::saveInFiles(std::string path) {
//	uint nSTATES = TPGMM::getNumSTATES();

	// save TPGMM
	tpgmm->saveInFiles(path);

	// save duration components
	mat durMu(1, nSTATES);
	mat durSigma(1, nSTATES);

	for(uint i=0; i<nSTATES; i++){
//			priors(0,i) = getPRIORS()(i);
		durMu(0,i) = getDurMU(i)(0);
		durSigma(0,i) = getDurSIGMA(i)(0);
//			for(uint j=0; j<nVARS; j++){
//				mu(j,i) = getMU(i)(j);
//				for(uint k=0; k<nVARS; k++)
//					sigma(j,k + i*(nVARS)) = getSIGMA(i)(j,k);
//			}
	}

//		priors.save(path + "priors.txt", raw_ascii);
//		mu.save(path + "mu.txt", raw_ascii);
//		sigma.save(path + "sigma.txt", raw_ascii);
//		transition.save(path + "transition.txt", raw_ascii);
	durMu.save(path + "durMu.txt", raw_ascii);
	durSigma.save(path + "durSigma.txt", raw_ascii);

	//save transition matrix
	TransitionMatrix.save(path + "transitionMatrix.txt", raw_ascii);

	std::ofstream Varfile(path + "varnames.txt");
	for (int i=0; i < tpgmm->getNumVARS(); i++){
		Varfile << "x" << i << " ";
	}
	Varfile.close();
	cout << "Saving in path: "<< path << endl;

//
//	uint nVARS = tpgmm->getNumVARS();
//	mat priors(1, nSTATES);
//	mat mu(nVARS, nSTATES);
//	mat sigma(nVARS, nVARS*nSTATES);
//	mat transition(nSTATES, nSTATES);
//	mat durMu(1, nSTATES);
//	mat durSigma(1, nSTATES);
//
//	for(uint i=0; i<nSTATES; i++){
////			priors(0,i) = getPRIORS()(i);
//		durMu(0,i) = getDurMU(i)(0);
//		durSigma(0,i) = getDurSIGMA(i)(0);
////			for(uint j=0; j<nVARS; j++){
////				mu(j,i) = getMU(i)(j);
////				for(uint k=0; k<nVARS; k++)
////					sigma(j,k + i*(nVARS)) = getSIGMA(i)(j,k);
////			}
//	}
//	transition = this->getTRANSITION();
//
//	cout << "Saving in path: "<< path << endl;
////		priors.save(path + "priors.txt", raw_ascii);
////		mu.save(path + "mu.txt", raw_ascii);
////		sigma.save(path + "sigma.txt", raw_ascii);
////		transition.save(path + "transition.txt", raw_ascii);
//	durMu.save(path + "durMu.txt", raw_ascii);
//	durSigma.save(path + "durSigma.txt", raw_ascii);
//
////		hsmm_transition.save(path + "hsmm_transition.txt", raw_ascii);
////		hsmm_transition_ticks.save(path + "hsmm_transition_ticks.txt", raw_ascii);
////		hsmm_priors.save(path + "hsmm_priors.txt", raw_ascii);
//	hsmm_priors_ticks.save(path + "hsmm_priors_ticks.txt", raw_ascii);
//	saveTPTtransitionMatrix(path);
}




//void TPHSMM::saveInFiles(std::string path) {
////	uint nSTATES = TPGMM::getNumSTATES();
//	uint nVARS = tpgmm->getNumVARS();
//	mat priors(1, nSTATES);
//	mat mu(nVARS, nSTATES);
//	mat sigma(nVARS, nVARS*nSTATES);
//	mat transition(nSTATES, nSTATES);
//	mat durMu(1, nSTATES);
//	mat durSigma(1, nSTATES);
//
//	for(uint i=0; i<nSTATES; i++){
////			priors(0,i) = getPRIORS()(i);
//		durMu(0,i) = getDurMU(i)(0);
//		durSigma(0,i) = getDurSIGMA(i)(0);
////			for(uint j=0; j<nVARS; j++){
////				mu(j,i) = getMU(i)(j);
////				for(uint k=0; k<nVARS; k++)
////					sigma(j,k + i*(nVARS)) = getSIGMA(i)(j,k);
////			}
//	}
//	transition = this->getTRANSITION();
//
//	cout << "Saving in path: "<< path << endl;
////		priors.save(path + "priors.txt", raw_ascii);
////		mu.save(path + "mu.txt", raw_ascii);
////		sigma.save(path + "sigma.txt", raw_ascii);
////		transition.save(path + "transition.txt", raw_ascii);
//	durMu.save(path + "durMu.txt", raw_ascii);
//	durSigma.save(path + "durSigma.txt", raw_ascii);
//
////		hsmm_transition.save(path + "hsmm_transition.txt", raw_ascii);
////		hsmm_transition_ticks.save(path + "hsmm_transition_ticks.txt", raw_ascii);
////		hsmm_priors.save(path + "hsmm_priors.txt", raw_ascii);
//	hsmm_priors_ticks.save(path + "hsmm_priors_ticks.txt", raw_ascii);
//	saveTPTtransitionMatrix(path);
//}
void TPHSMM::saveTPTtransitionMatrix(std::string path)
{
//	uint nSTATES = TPGMM::getNumSTATES();
	mat nTrans(nSTATES,nSTATES);
	mat mu(nSTATES*nSTATES,nTPS);
	cube sigma(nSTATES*nSTATES,nTPS,nTPS);

	for(int i=0;i<nSTATES;i++) {
		for(int j=0;j<nSTATES;j++) {
			nTrans(i,j)= TPTransitionMatrix[i][j].nTrans;
			for(int i_T=0;i_T<nTPS;i_T++) {
				mu(j+i*nSTATES,i_T) = TPTransitionMatrix[i][j].transParam->getMU()(i_T);
				for(int j_T=0;j_T<nTPS;j_T++) {
					sigma(j+i*nSTATES,i_T,j_T) = TPTransitionMatrix[i][j].
							transParam->getSIGMA()(i_T,j_T);
				}
			}
		}
	}
	nTrans.save(path + "nTrans_tp_transition.txt", raw_ascii);
	mu.save(path + "mu_tp_transition.txt", raw_ascii);
	sigma.save(path + "sigma_tp_transition.txt", arma_binary);
}

}
