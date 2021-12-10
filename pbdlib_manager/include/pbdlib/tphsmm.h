/*
 * tphsmm.h
 *
 *  Created on: Sep 15, 2015
 *      Author: Ajay Tanwani
 */

#ifndef TPHSMM_H_
#define TPHSMM_H_

#include "armadillo"
#include "pbdlib/tpgmm.h"
#include "pbdlib/taskparameters.h"
#include "pbdlib/tpdemonstration.h"
#include "pbdlib/hsmm.h"
#include "lqr.h"
#include "assert.h"

using namespace arma;

namespace pbdlib
{
// class representing parametrized transition between states
class TPTRANSITION
{
public:
    TPTRANSITION(uint _nTPS){
        nTPS = _nTPS;
        colvec defMu = zeros(_nTPS);
        mat defSigma = eye(_nTPS,_nTPS);
        nTrans =0;
        transParam = new GaussianDistribution(defMu,defSigma);
    }
    TPTRANSITION(uint _nTPS,vec Param,float _minSigma){
        nTPS = _nTPS;
        mat minSIGMA = _minSigma * eye(_nTPS,_nTPS);
        nTrans =1;
        transParam = new GaussianDistribution(Param,minSIGMA);
    }
    /*
    ~TPTRANSITION(){
        delete transParam;
    }
     */
    void addTransition(vec _trParam,float _minSigma)
    {
        nTrans++;

        colvec MuTmp = 1.0/nTrans * ((nTrans-1.0)*transParam->getMU() + _trParam);
        mat SigmaTmp;
        if(nTrans>1){
            SigmaTmp = 1.0/nTrans * ((nTrans-1.0)*transParam->getSIGMA() +
                    (double)(nTrans/(nTrans-1.0)) * ((_trParam-MuTmp)*(_trParam-MuTmp).t()));
        }
        else SigmaTmp = _minSigma * eye(nTPS,nTPS);
        transParam->setMU(MuTmp);
        transParam->setSIGMA(SigmaTmp);
    }
    GaussianDistribution *transParam; // reprensenting the parameters where the transition is likely to occur
    uint nTrans; // number of transition detected
    uint nTPS;
};

class TPHSMM : public HSMM {

private:

	// myTPs -> current Task Frames of a demonstration myTPs[i].A, myTPs[i].b

	uint nbVARPOS;
	LQR *lqr;
	TPGMM *tpgmm;
	TaskParameters *taskparam;
	GMM_Model *ProdGauss;
	colvec CurrentState, InitialState;
	uword StateSequence[200];
	std::vector<TaskParameter> cTaskParam;

	mat AlphaHSMM;

	uint pred_length;
	mat rData;
	uint SamplingMethod, HorizonMethod, SimMethod;

	double RegularizationConstLQR;


    mat TransCount;
public:
    uint nTPS;
    float minSigma;
//    uint nSTATES;

    std::vector< std::vector<TPTRANSITION> > TPTransitionMatrix;

public:
	//TPHSMM();
	//TPHSMM(std::vector<TPDemonstration>& demos, uint nSTATES);
	TPHSMM(uint nVARS, uint nSTATES, uint _nTPs);
	TPHSMM(uint nVARS, uint nSTATES, uint _nTPs, uint nbVARPOS, double dt, double rf);
	TPHSMM(std::vector<GMM_Model> GMMS, uint _nVARS, uint _nSTATES, uint _nTPs);
	TPHSMM(std::vector<GMM_Model> GMMS, mat transition, std::vector<GaussianDistribution> components, uint _nVARS, uint _nSTATES, uint _nTPs);

	// For online learning
	TPHSMM(uint _nVARS,uint _nTPs, float _minSigma);
	TPHSMM(std::string path,uint _nSTATES,uint _nVARS,uint _nTPs, float _minSigma);



//	TPHSMM(std::string PriorsFileName,
//					std::string VarsFileName,
//					std::string MuFilePrefix,
//					std::string SigmaFilePrefix,
//					const std::string &transition_path,
//					const std::string &durMu_path,
//					const std::string &durSigma_path);
	virtual ~TPHSMM();

	void loadTPHSMMfromFiles(std::string PriorsFileName,
			std::string VarsFileName,
			std::string MuFilePrefix,
			std::string SigmaFilePrefix,
			const std::string &transition_path,
			const std::string &durMu_path,
			const std::string &durSigma_path);

	void setCurrentTaskFrameFromFile(std::string FramePath);
	void setCurrentTaskFrame(std::vector<TaskParameter> TaskParameters);

	void setCurrentState(colvec state);
	void setInitialState(colvec init_state);
	void setInitialState(GMM_Model& ProdGauss);

	void setSamplingMethod(uint SamplingMethod);
	void setLQRHorizonMethod(uint HorizonMethod);
	void setPredictionLength(uint PredictionLength);
	void setSimulationMethod(uint SimMethod);
	void setScalingR(double rf);

	void setRegularizationConstLQR(double RC);

	std::vector<TaskParameter>& getTaskParameters();

//	uint getNumSTATES();

	void init_TPHSMM_LQR(double dt, double rf);

	colvec& getInitialState();
	colvec& getCurrentState();

	mat TPHSMMController();
	mat TPHSMMController(std::vector<TaskParameter> cTaskParam, uint pred_length);
	mat TPHSMMController(std::vector<TaskParameter> cTaskParam, colvec init_state, uint pred_length);

	mat HSMMLQR(GMM_Model& GMM, colvec init_state, uint pred_length);

	void printGMM(GMM_Model& GMM);
	void printGaussian(GaussianDistribution& GaussPr);

//	std::vector<GMM_Model>& getGMMS();
//	GMM_Model& getGMMS(uint id);

//	GMM_Model* getTransformedGMM(std::vector<TaskParameter> _TPs, EMatrixType AType = OTHER );// [Calinon, 2012] Eq. (3) - Prod. of linearly transformed Gaussians
//	GMM_Model* getTransformedGMM(TaskParameters _TPs, EMatrixType AType = OTHER );// [Calinon, 2012] Eq. (3) - Prod. of linearly transformed Gaussians

//	void setVARSNames(const std::vector<std::string>& vars);
//	std::vector<std::string>& getVARSNames();

	TPGMM* get_TPGMM();
	GMM_Model* get_ProdGMM(std::vector<TaskParameter> cTaskParam);
	LQR* get_LQR();

	// update the transition matrix given the parametrized trans. matrix and the parameters
    void updateTransitionMat(vec _trParam);

    void updateDuration(uint _idxSTATE,uint _timeStepCNT);

    void mergeTransition(uint, uint);

    double getTransSimilarity(uint,uint);// get a measure of similarity between two states

    void saveInFiles(std::string path);
	void saveTPTtransitionMatrix(std::string path);

    void loadFromFiles(std::string path);
	// to call when a new transition is detected
    void addTransition(uint _prevSTATE, uint _newSTATE, vec _trParam,bool startFlag, uint _timeStepCNT);

};

}

#endif /* TPHSMM_H_ */

