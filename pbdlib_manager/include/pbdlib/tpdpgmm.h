//
// Created by pignate on 22.10.15.
//

#ifndef PBDLIB_TPDPGMM_H
#define PBDLIB_TPDPGMM_H

#define P_1VAR 0.6065

#include "armadillo"
#include "pbdlib/gmm.h"
#include "pbdlib/tpgmm.h"
#include "pbdlib/taskparameters.h"
#include "pbdlib/tphsmm.h"

using namespace arma;

namespace pbdlib {

#define REALMIN 2.2251e-200
#define REALMAX 1.7977e200

class TPDPGMM: public TPGMM{

private:
    uint nbPoints;// nb of points already added to the model
    uint prevState;
    float lambda;
	float lambda2;
	float lambda3;
    float minSigma;
    uint timeStepCNT; //count the time we stayed in the same states

    mat framePRIORS; // prior on the frame in which the state was selected [nSTATES x NTaskparameters+1]
    bool framePRIORSProd; // using or not priors on frame to make the product
    TPHSMM* tpTrans;

public:
    TPDPGMM(uint nVARS, uint nSTATES, uint _nTPs, uint /* nTPS for transition mat */_nTPsTr,
            float _lambda=50.f,float _minSigma = 2E2)
            :TPGMM(nVARS,nSTATES,_nTPs)
    {
        lambda = _lambda;
		lambda3 = _lambda;
		lambda2 = _lambda;
        nbPoints = 0;
        minSigma = _minSigma;
        prevState = 0;

        timeStepCNT =0;
        framePRIORSProd = true;

        tpTrans = new TPHSMM(nVARS,_nTPsTr,_lambda*10);

    }
	TPDPGMM(std::string path,uint nVARS, uint _nTPs, uint _nTPsTr,
			float _lambda=50.f,float _minSigma = 2E2)
			:TPGMM(path,_nTPs)
	{
		lambda = _lambda;
		lambda2 = _lambda;
		lambda3 = _lambda;
		colvec nbPts;
		nbPts.load(path + "TPGMM_nbPoints.txt",raw_ascii);
		nbPoints = nbPts(0);
		minSigma = _minSigma;
		prevState = 0;

		timeStepCNT =0;
		framePRIORSProd = true;

		framePRIORS.load(path + "TPGMM_frame_PRIORS.txt",raw_ascii);

		tpTrans = new TPHSMM(path,nSTATES,nVARS,_nTPsTr,_lambda*10);

	}
    void  addDatapoints(mat _dataPoint,vec _trParam,bool startFlag=false,bool endFlag=false,
                        float _lambda= 0.f,float _minSigma = 0.f);
    void  addDatapoints_d(mat _dataPoint,vec _trParam,bool startFlag=false,bool endFlag=false,
            float _lambda= 0.f,float _lambda2=-1.f,float _lambda3=-1.f,float _minSigma = 0.f,int type=0,
						  bool sticky = false);

    uint getNumPoints();

    GMM_Model* getProdGMM();

    void useFramePRIORS(bool _use){
        framePRIORSProd = _use;
    }

    void updateTransition(vec _trParam);

    bool reduceStates(); // search for similar states
    bool reduceStates_d(); // search for similar states
    void mergeStates(uint,uint); // merge states

    mat predictForwardVariable(uint _N,uint startState);

    mat getTransitionMatrix();
    void setTransitionMatrix(mat tm);

    GMM_Model* getTransformedGMM(std::vector<TaskParameter> _TPs, EMatrixType AType = OTHER );
    GMM_Model* getTransformedGMM(TaskParameters _TPs, EMatrixType AType = OTHER );// [Calinon, 2012] Eq. (3) - Prod. of linearly transformed Gaussians

    void saveInFiles(std::string path);

    void loadFromFiles(std::string path);


    HSMM* getHSMM(){return tpTrans;}
    void setDurationComponents(const std::vector<GaussianDistribution>& components){
//      tpTrans = new TPHSMM(nVARS,_nTPsTr,_lambda*10);
			tpTrans = new TPHSMM(this->getNumVARS(), components.size(), this->getNumFRAMES());
    	tpTrans->setDurationCOMPONENTS(components);
    	tpTrans->initializeFwdCalculation();
    }


};

}

#endif //PBDLIB_TPDPGMM_H
