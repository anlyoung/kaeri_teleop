//
// Created by pignate on 22.10.15.
//

#include <stdio.h>
#include "pbdlib/tpdpgmm.h"
#include <pbdlib/gmm.h>
#include "math.h"

using namespace arma;
using namespace std;

namespace pbdlib {

void TPDPGMM::addDatapoints(mat _dataPoint,vec _trParam, bool startFlag, bool endFlag, float _lambda,float _minSigma ) {
    if (_lambda != 0.0)
        lambda = _lambda;
    if (_minSigma != 0.0)
        minSigma = _minSigma;
	//cout<<minSigma<<endl;
    uint curState;
    if (nbPoints == 0)
    {
        std::vector<GaussianDistribution> comps;
        mat minSIGMA = eye(nVARS,nVARS)*minSigma;

        GMM_Model tmpGMM(nSTATES,nVARS);
       	colvec dataFrame;

        for(uint i=0;i<nTaskParameters;i++)
        {
            dataFrame = colvec(_dataPoint.col(i));
            //cout<<dataFrame<<endl;
            GaussianDistribution componentTmp(dataFrame,minSIGMA);
            comps.push_back(componentTmp);

            tmpGMM.setCOMPONENTS(comps);
            GMMS.push_back(tmpGMM);

            //cout<<GMMS[i].getMU(0)<<endl;
            //cout<<nSTATES<<endl;

            comps.clear();
        }
		PRIORS = 1;

        framePRIORS = ones(1,nTaskParameters+1);
        framePRIORS(0,nTaskParameters) = 1;
        curState = 0;
    }
    else
    {
        colvec dataFrame;

        mat lik(nTaskParameters+1,nSTATES+1);

        mat SigmaTmp = eye(nVARS,nVARS)*lambda*lambda;
        colvec MuTmp = zeros(nVARS);

        GaussianDistribution cluster(MuTmp,SigmaTmp);

        for(uint k=0; k<nSTATES;k++)
        {
            for(uint m =0;m<nTaskParameters;m++)
            {
                dataFrame = colvec(_dataPoint.col(m));
                //likelihood
                cluster.setMU(GMMS[m].getMU(k));
                lik(m,k) = cluster.getPDFValue(dataFrame)(0);
            }

            cluster.setMU(prodGMM->getMU(k));
//            lik(nTaskParameters,k) = cluster.getPDFValue(_dataPoint.col(nTaskParameters))(0);
            // disable assignement for gaussian product
			lik(nTaskParameters,k) = 0.0;
        }

        double probLim = 1./pow(lambda,nVARS)/sqrt(pow(2*PI,nVARS))*P_1VAR;

        for(uint m =0;m<nTaskParameters+1;m++)
        {
            lik(m,nSTATES) = probLim;
        }
        //cout << lik <<endl;
        // store likelihood without the frame priors to update the frame prior ( avoid self-reinforcement )
        mat likTmp = lik;

        // multiplying prob to be in frame with prob to be in cluster -- comment this line to see the effect
        lik.cols(0,nSTATES-1) %= framePRIORS.t();

        //cout<<probFrame<<endl;
        //cout<<framePRIORS<<endl;

        uvec vecMaxIdx = find(lik == max(max(lik)),1);

        //cout<<d<<endl;
        uint maxIndex = vecMaxIdx(0)/(nTaskParameters+1);

        umat frameAssign = (likTmp.col(maxIndex)>probLim);
        //cout<< frameAssign<<endl;

        uvec frameIndex = find(frameAssign);

        curState = maxIndex;
//        cout<<framePRIORS.row(maxIndex)<<endl;

        if(maxIndex==nSTATES)
        {
            rowvec priorsTmp = zeros(1,nSTATES+1);
            priorsTmp.cols(0,nSTATES-1) = this->getPRIORS();
            priorsTmp(nSTATES) = 1./nbPoints;
            priorsTmp = priorsTmp/arma::norm(priorsTmp,1);
            PRIORS = priorsTmp;

            nSTATES++;
            // new states : resize priors frame Priors
            mat framePRIORSTmp = framePRIORS;
            framePRIORS = ones(nSTATES,nTaskParameters+1);
            framePRIORS.rows(0,nSTATES-2) = framePRIORSTmp;
            framePRIORS(nSTATES-1,nTaskParameters) = 1.0;

            for(uint k=0;k<nTaskParameters;k++)
            {
                mat minSIGMA = eye(nVARS, nVARS) * minSigma;
                dataFrame = colvec(_dataPoint.col(k));
                GaussianDistribution componentTmp(dataFrame,minSIGMA);

                std::vector<GaussianDistribution> prevComps = GMMS[k].getCOMPONENTS();

                prevComps.push_back(componentTmp);

                GMMS[k] = GMM_Model(nSTATES,nVARS);
                GMMS[k].setCOMPONENTS(prevComps);
                //GMMS[k].setCOMPONENTS(prevComps);
            }
        }
        else
        {
            double PriorsTmp = 1./nbPoints + PRIORS[maxIndex];

            uvec maxIndexVec(1);
            maxIndexVec(0) = maxIndex;

            for(uint k=0;  k<nTaskParameters;k++)
            {
                colvec P = colvec(_dataPoint.col(k));
                colvec MuTmp = 1./PriorsTmp*(PRIORS[maxIndex]*GMMS[k].getMU(maxIndex)+P/nbPoints);
                mat SigmaTmp = PRIORS[maxIndex]/PriorsTmp*(GMMS[k].getSIGMA(maxIndex)+
                        (GMMS[k].getMU(maxIndex)-MuTmp)*(GMMS[k].getMU(maxIndex)-MuTmp).t()) +
                        1./(nbPoints*PriorsTmp)*(minSigma*eye(nVARS,nVARS)+(P-MuTmp)*(P-MuTmp).t());

                GMMS[k].setMU(maxIndex,MuTmp);
                GMMS[k].setSIGMA(maxIndex,SigmaTmp);
            }

			// the probabilty to be in both frames is added to the probabilty to be in one state and not the other
			framePRIORS.row(maxIndex) =(PRIORS[maxIndex]/PriorsTmp) *framePRIORS.row(maxIndex) +
					(1.0/((nbPoints)*PriorsTmp))* (frameAssign.t()%ones(1,nTaskParameters+1));

            PRIORS[maxIndex] = PriorsTmp;
            PRIORS = PRIORS/arma::norm(PRIORS,1);
        }
    }
    timeStepCNT++;
    if(prevState!=curState||endFlag){ // transition detected

		tpTrans->addTransition(prevState,curState,_trParam,startFlag,timeStepCNT);
		if(endFlag&&prevState!=curState)
		{
			tpTrans->addTransition(curState,curState,_trParam,startFlag,timeStepCNT);
		}

        tpTrans->updateTransitionMat(_trParam);
        //cout<<timeStepCNT<<endl;
        timeStepCNT =0;
    }
	//cout<<framePRIORS<<endl;
    setAuxiliaryVarsProdGauss();
    //getTransformedGMM()
    prevState = curState;
    nbPoints++;
}

void TPDPGMM::addDatapoints_d(mat _dataPoint,vec _trParam, bool startFlag, bool endFlag, float _lambda,
							  float _lambda2, float _lambda3,float _minSigma, int type ,bool sticky) {
	if (_lambda != 0.0)
		lambda = _lambda;
	if (_lambda2 != -1.f)
		lambda2 = _lambda2;
    if (_lambda3 != -1.f)
		lambda3 = _lambda3;
	if (_minSigma != 0.0)
		minSigma = _minSigma;

	uint curState;
	if (nbPoints == 0)
	{
		std::vector<GaussianDistribution> comps;
		mat minSIGMA = eye(nVARS,nVARS)*minSigma;

		GMM_Model tmpGMM(nSTATES,nVARS);
		colvec dataFrame;

		for(uint i=0;i<nTaskParameters;i++)
		{
			dataFrame = colvec(_dataPoint.col(i));
			//cout<<dataFrame<<endl;
			GaussianDistribution componentTmp(dataFrame,minSIGMA);
			comps.push_back(componentTmp);

			tmpGMM.setCOMPONENTS(comps);
			GMMS.push_back(tmpGMM);

			//cout<<GMMS[i].getMU(0)<<endl;
			//cout<<nSTATES<<endl;

			comps.clear();
		}
		PRIORS = 1;

		framePRIORS = ones(1,nTaskParameters+1);
		framePRIORS(0,nTaskParameters) = 1;
		curState = 0;
	}
	else
	{
		colvec dataFrame;

		mat lik(nTaskParameters+1,nSTATES+1);
		mat dist(nTaskParameters+1,nSTATES+1);

		mat SigmaTmp = eye(nVARS,nVARS)*lambda*lambda;
		colvec MuTmp = zeros(nVARS);

		GaussianDistribution cluster(MuTmp,SigmaTmp);

		mat transTmp = tpTrans->getTRANSITION();
		transTmp.elem( find_nonfinite(transTmp)).zeros();//change NaN to zero

		// compute the probability to go out of the state given the HSMM duration
//		mat timeVal(1,1);
//		timeVal(0,0) = (double)timeStepCNT;
//		double transOutProb = tpTrans->getDurationCOMPONENTS(prevState).getCDFValue(timeVal)(0);

		double transOutProb = erf(((double)timeStepCNT-tpTrans->getDurMU(prevState)(0))/tpTrans->getDurSIGMA(prevState)(0,0));
		transOutProb = (1.0 + transOutProb)/2.0;
		// cout << "Transition out prob :"<<transOutProb<<endl;

		for(uint k=0; k<nSTATES;k++)
		{
			for(uint m =0;m<nTaskParameters;m++)
			{
				dataFrame = colvec(_dataPoint.col(m));
				//likelihood
//				cluster.setMU(GMMS[m].getMU(k));
//				lik(m,k) = cluster.getPDFValue(dataFrame)(0);

				if(k==prevState||startFlag)
				{

					if(sticky)
						dist(m,k) = arma::norm(GMMS[m].getMU(k) - dataFrame,2)+transOutProb*lambda3;
					else if(transTmp.n_rows>0&&sum(transTmp.row(prevState))==0.0f)
						dist(m,k) = arma::norm(GMMS[m].getMU(k) - dataFrame,2)+lambda3;
					else
						dist(m,k) = arma::norm(GMMS[m].getMU(k) - dataFrame,2);
//					dist(m,k) = arma::norm(GMMS[m].getMU(k) - dataFrame,2);
				}
				else
				{	if(sticky)
					{
						dist(m,k) = arma::norm(GMMS[m].getMU(k) - dataFrame,2)+
							lambda3*(1.0-transOutProb*transTmp(prevState,k));
					}
					else
					{
						dist(m,k) = arma::norm(GMMS[m].getMU(k) - dataFrame,2)+
									lambda3*(1.0-transTmp(prevState,k));
					}

				}
			}

//			cluster.setMU(prodGMM->getMU(k));
//            lik(nTaskParameters,k) = cluster.getPDFValue(_dataPoint.col(nTaskParameters))(0);
			// disable assignement for gaussian product
//			lik(nTaskParameters,k) = 0.0;
			dist(nTaskParameters,k) = 1000*lambda;
		}

//		double probLim = 1./pow(lambda,nVARS)/sqrt(pow(2*PI,nVARS))*P_1VAR;

		for(uint m =0;m<nTaskParameters+1;m++)
		{
//			lik(m,nSTATES) = probLim;
//			if(sum(transTmp.row(prevState))==0)
//				dist(m,nSTATES) = lambda;
//			else
			dist(m,nSTATES) = lambda+lambda3;
		}
		dist(nTaskParameters,nSTATES) = 1000*lambda;
		// cout<<"Time in current state : "<< timeStepCNT << " / " << tpTrans->getDurMU(prevState)<<endl;

		//cout << lik <<endl;
		// store likelihood without the frame priors to update the frame prior ( avoid self-reinforcement )
//		mat likTmp = lik;
//		cout << transTmp<<endl;
//		cout << dist <<endl;
//		cout << framePRIORS.t()<<endl;

		uint maxIndex;
		if(type==0)
		{
			// multiplying prob to be in frame with prob to be in cluster -- comment this line to see the effect
//			lik.cols(0, nSTATES - 1) %= framePRIORS.t();
			dist.cols(0, nSTATES - 1) += lambda2* (1. - framePRIORS.t());
			//cout << dist << endl;
			//cout<<probFrame<<endl;
			//cout<<framePRIORS<<endl;


	//		uvec vecMaxIdx = find(lik == max(max(lik)),1);
			uvec vecMaxIdx = find(dist == min(min(dist)), 1);

			//cout<<d<<endl;
			maxIndex = vecMaxIdx(0) / (nTaskParameters + 1);
		}
		else if(type==1)
		{
			mat sumDist = sum(dist,0);
			cout <<sumDist<<endl;

			uvec vecMaxIdx = find(sumDist == min(min(sumDist)), 1);
			//cout<<d<<endl;
			maxIndex = vecMaxIdx(0);
		}
		umat frameAssign = (dist.col(maxIndex)<lambda+lambda3);
//		umat frameAssign = (dist.col(maxIndex)<lambda);
//		cout<< frameAssign<<endl;

		//uvec frameIndex = find(frameAssign);

		curState = maxIndex;

		if(maxIndex==nSTATES)
		{
			rowvec priorsTmp = zeros(1,nSTATES+1);
			priorsTmp.cols(0,nSTATES-1) = this->getPRIORS();
			priorsTmp(nSTATES) = 1./nbPoints;
			priorsTmp = priorsTmp/arma::norm(priorsTmp,1);
			PRIORS = priorsTmp;

			nSTATES++;
			// new states : resize priors frame Priors
			mat framePRIORSTmp = framePRIORS;
			framePRIORS = ones(nSTATES,nTaskParameters+1);
			framePRIORS.rows(0,nSTATES-2) = framePRIORSTmp;
			framePRIORS(nSTATES-1,nTaskParameters) = 1.0;

			for(uint k=0;k<nTaskParameters;k++)
			{
				mat minSIGMA = eye(nVARS, nVARS) * minSigma;
				dataFrame = colvec(_dataPoint.col(k));
				GaussianDistribution componentTmp(dataFrame,minSIGMA);

				std::vector<GaussianDistribution> prevComps = GMMS[k].getCOMPONENTS();

				prevComps.push_back(componentTmp);

				GMMS[k] = GMM_Model(nSTATES,nVARS);
				GMMS[k].setCOMPONENTS(prevComps);
				//GMMS[k].setCOMPONENTS(prevComps);
			}
		}
		else
		{
			double PriorsTmp = 1./nbPoints + PRIORS[maxIndex];

			uvec maxIndexVec(1);
			maxIndexVec(0) = maxIndex;

			for(uint k=0;  k<nTaskParameters;k++)
			{
				colvec P = colvec(_dataPoint.col(k));
				colvec MuTmp = 1./PriorsTmp*(PRIORS[maxIndex]*GMMS[k].getMU(maxIndex)+P/nbPoints);
				mat SigmaTmp = PRIORS[maxIndex]/PriorsTmp*(GMMS[k].getSIGMA(maxIndex)+
														   (GMMS[k].getMU(maxIndex)-MuTmp)*(GMMS[k].getMU(maxIndex)-MuTmp).t()) +
							   1./(nbPoints*PriorsTmp)*(minSigma*eye(nVARS,nVARS)+(P-MuTmp)*(P-MuTmp).t());

				GMMS[k].setMU(maxIndex,MuTmp);
				GMMS[k].setSIGMA(maxIndex,SigmaTmp);
			}

			// the probabilty to be in both frames is added to the probabilty to be in one state and not the other
			framePRIORS.row(maxIndex) =(PRIORS[maxIndex]/PriorsTmp) *framePRIORS.row(maxIndex) +
									   (1.0/((nbPoints)*PriorsTmp))* (frameAssign.t()%ones(1,nTaskParameters+1));

			PRIORS[maxIndex] = PriorsTmp;
			PRIORS = PRIORS/arma::norm(PRIORS,1);
		}
	}
	timeStepCNT++;
	if(prevState!=curState||endFlag){ // transition detected

		tpTrans->addTransition(prevState,curState,_trParam,startFlag,timeStepCNT);
		if(endFlag&&prevState!=curState)
		{
			tpTrans->addTransition(curState,curState,_trParam,startFlag,timeStepCNT);
		}

		tpTrans->updateTransitionMat(_trParam);
		//cout<<timeStepCNT<<endl;
		timeStepCNT =0;
	}
	//cout<<framePRIORS<<endl;
	setAuxiliaryVarsProdGauss();
	//getTransformedGMM()
	prevState = curState;
	nbPoints++;
}


GMM_Model* TPDPGMM::getProdGMM()
{
    return prodGMM;
}

uint TPDPGMM::getNumPoints()
{
    return nbPoints;
}
mat TPDPGMM::getTransitionMatrix()
{
	return tpTrans->getTRANSITION();
}
void TPDPGMM::setTransitionMatrix(mat tm)
{
	tpTrans->setTRANSITION(tm);
}

bool TPDPGMM::reduceStates()
{
//    cube div = zeros(nSTATES, nSTATES, nTaskParameters); // Kullback-Leibler divergence
    cube like = zeros(nSTATES, nSTATES, nTaskParameters+1);
    mat transProb = zeros(nSTATES,nSTATES); // probability to have the sames transitions

	mat probFrame = framePRIORS;


	for (uint k = 0; k < nSTATES; k++)
    {
        for (uint l = 0; l < k; l++)
		{
			transProb(k,l) = tpTrans->getTransSimilarity(k,l);

			for (uint m = 0; m < nTaskParameters; m++)
			{
				//div(k,l,m)=GMMS[m].getCOMPONENTS(k).getKLdiv(GMMS[m].getCOMPONENTS(l));
                /*
				like(k, l, m) = transProb(k,l)*sqrt(probFrame(k, m) *probFrame(l, m))
								* exp(-0.5 * arma::norm(GMMS[m].getMU(k) - GMMS[m].getMU(l)) / lambda);
                                */
                like(k, l, m) = sqrt(probFrame(k, m) *probFrame(l, m))
                                * exp(-0.5 * arma::norm(GMMS[m].getMU(k) - GMMS[m].getMU(l)) / lambda);
				/*
				like(k, l, m) = sqrt(probFrame(k, m) *probFrame(l, m))
				 * exp(-0.5 * arma::norm(GMMS[m].getMU(k) - GMMS[m].getMU(l)) / lambda);
				*/
			}
			like(k, l, nTaskParameters) = sqrt(probFrame(k, nTaskParameters) * probFrame(l, nTaskParameters)) *
										  exp(-0.5 * arma::norm(prodGMM->getMU(k) - prodGMM->getMU(l)) / lambda);
			// to avoid merging in absolute reference frame
			like(k, l, nTaskParameters) = 0;
		}

    }
//	cout<<like<<endl;
//	cout<<transProb<<endl;

	uword k,l,m;
    double maxProb = like.max(k,l,m);

    if (maxProb > P_1VAR)
    {
        cout << "States " <<k << " and " << l << " will be merged."<< endl;
        mergeStates(k,l);
		return true;
    }
	else
		return false;
}

bool TPDPGMM::reduceStates_d()
{
//    cube div = zeros(nSTATES, nSTATES, nTaskParameters); // Kullback-Leibler divergence
	cube like = zeros(nSTATES, nSTATES, nTaskParameters+1);
	cube dist = ones(nSTATES, nSTATES, nTaskParameters+1);
	dist.fill(2*lambda);
	mat transProb = zeros(nSTATES,nSTATES); // probability to have the sames transitions

	mat probFrame = framePRIORS;


	for (uint k = 0; k < nSTATES; k++)
	{
		for (uint l = 0; l < k; l++)
		{

			transProb(k,l) = tpTrans->getTransSimilarity(k,l);

			for (uint m = 0; m < nTaskParameters; m++)
			{

	//				like(k, l, m) = sqrt(probFrame(k, m) *probFrame(l, m))
	//								* exp(-0.5 * arma::norm(GMMS[m].getMU(k) - GMMS[m].getMU(l)) / lambda);
				dist(k,l,m) = arma::norm(GMMS[m].getMU(k) - GMMS[m].getMU(l))
							  +(1.0-framePRIORS(k, m))*(1.0-framePRIORS(l, m))*lambda3;
			}
	//			like(k, l, nTaskParameters) = sqrt(probFrame(k, nTaskParameters) * probFrame(l, nTaskParameters)) *
	//										  exp(-0.5 * arma::norm(prodGMM->getMU(k) - prodGMM->getMU(l)) / lambda);
	//			// to avoid merging in absolute reference frame
	//			like(k, l, nTaskParameters) = 0;
			dist(k,l,nTaskParameters) = 2*lambda;
		}

	}
//	cout<<like<<endl;
//	cout<<transProb<<endl;

	uword k,l,m;
//	double maxProb = like.max(k,l,m);
	float minDist = dist.min(k,l,m);
	//cout<< dist<<endl;

//	if (maxProb > P_1VAR)
	if (minDist < lambda)
	{
		cout << "States " <<k << " and " << l << " will be merged."<< endl;
		mergeStates(k,l);
		return true;
	}
	else
		return false;
}
    
void TPDPGMM::mergeStates(uint k, uint l)
{
    double PriorsTmp = PRIORS(k) + PRIORS(l);
    for (uint m = 0; m < nTaskParameters; m++)
    {
        colvec MuTmp = 1.0 / PriorsTmp * (PRIORS[k] * GMMS[m].getMU(k) + PRIORS[l] * GMMS[m].getMU(l));
        mat SigmaTmp = 1.0 / PriorsTmp * (PRIORS[k] * (GMMS[m].getSIGMA(k) +
                                                     (GMMS[m].getMU(k) - MuTmp) * (GMMS[m].getMU(k) - MuTmp).t())
                                        + PRIORS[l] * (GMMS[m].getSIGMA(l) +
                                                       (GMMS[m].getMU(l) - MuTmp) * (GMMS[m].getMU(l) - MuTmp).t()));
        GMMS[m].setMU(l, MuTmp);
        GMMS[m].setSIGMA(l, SigmaTmp);
        GMMS[m].deleteCOMPONENT(k);
    }
	tpTrans->mergeTransition(k,l);
	prodGMM->deleteCOMPONENT(k);
	framePRIORS.shed_row(k);
	PRIORS.shed_col(k);
	if (prevState == k)
		prevState = l;
	else if(prevState>k)
		prevState--;
    nSTATES--;

}
void TPDPGMM::updateTransition(vec _trParam){

    tpTrans->updateTransitionMat(_trParam);
//    cout<<tpTrans->getDurationCOMPONENTS(2).getMU()<<endl;
//    cout<<tpTrans->getTRANSITION()<<endl;
}
mat TPDPGMM::predictForwardVariable(uint _N, uint startState){

    tpTrans->setPRIORS(prodGMM->getPRIORS());
    tpTrans->initializeFwdCalculation();

    mat pred;
//    pred.set_size(tpTrans->getNumSTATES(),_N);
    pred.set_size(prodGMM->getNumSTATES(),_N);
    tpTrans->predictForwardVariableDeterministic(pred, startState);

	return pred;
}
GMM_Model* TPDPGMM::getTransformedGMM(TaskParameters _TPs, EMatrixType AType){

    return getTransformedGMM(_TPs.getTaskParameters(),AType);
}

GMM_Model* TPDPGMM::getTransformedGMM(std::vector<TaskParameter> _TPs, EMatrixType AType)
{
    double diagRegularizationFactor = 1E-4;	//	regularization factor to invert A*Sigma*A' which is often close to singular

    // Check size of Task taskparameters
    if (_TPs.size()!=nTaskParameters)
        throw std::invalid_argument("TPGMM::getTransformedProdGMM(std::vector<TaskParameter> _TPs, EMatrixType AType) Number of supplied task parameters is not consistent expected number of taskparameters");


    //Projections in the different candidate taskParameters (see Eq. (3) Calinon et al, Humanoids'2012 paper)
    if (AType==OTHER || nSTATES<2)
    {
        // Use implementation based on covariance matrices when type of matrix A is not specified or
        // the number of states in the GMM is less than 2
        for (uint i=0; i<nSTATES; i++){
            accMu = zeros(nVARS, 1);
            accLambda= zeros(nVARS, nVARS);
            for (uint m = 0; m < nTaskParameters ; m++)
            {
                // Projecting Zmu of state "i" at taskParameter "m"
                tmpMu = _TPs[m].A * GMMS[m].getMU(i) + _TPs[m].b;

                // Projecting Zsigma of state "i" at taskParameter "m"
                tmpLambda= inv(_TPs[m].A * GMMS[m].getSIGMA(i) * trans(_TPs[m].A) + eye(nVARS,nVARS)*diagRegularizationFactor);

                // Accumulate:
                accLambda+= tmpLambda;
                accMu+=  tmpLambda* tmpMu;
            }
            // Saving the Gaussian distribution for state "i"
            prodGMM->setMU(i,accLambda.i()*accMu);
            prodGMM->setLAMBDA(i,accLambda);
        }
        prodGMM->setPRIORS(this->PRIORS);
    }
    else
    {
        // Approach based on Precision matrices:
        // Requires less inverses

        // Pre-compute A^-1 and (A^T)^-1.
        // First select method based on type of matrix A:
        switch (AType)
        {
            case ORTHONORMAL:
                for (uint m =0; m<nTaskParameters; m++)
                {
                    invA[m] = _TPs[m].A.t();
                    invAT[m] = _TPs[m].A;
                }
                break;
            case SYMMETRIC:
                for (uint m =0; m<nTaskParameters; m++)
                {
                    invA[m] = _TPs[m].A.i();
                    invAT[m] = invA[m];
                }
                break;
            case INVERTIBLE:
                for (uint m =0; m<nTaskParameters; m++)
                {
                    invA[m] = _TPs[m].A.i();
                    invAT[m] = inv(_TPs[m].A.t());
                }
                break;
            default:
                break;
        }

        mat refPRIORS;

        if(framePRIORSProd) {
            refPRIORS = framePRIORS.cols(0, nTaskParameters - 1);

            // the probabilty to be in both frames is added to the probabilty to be in one state and not the other
            //refPRIORS = refPRIORS + repmat(framePRIORSbis.col(nTaskParameters), 1, nTaskParameters);

            // to test but better without
            //refPRIORS = refPRIORS / repmat(sum(refPRIORS,1),1,nTaskParameters);
            //cout << refPRIORS << endl;
        }

        // Compute product of Gaussian
        for (uint i=0; i<nSTATES; i++){
            accMu = zeros(nVARS, 1);
            accLambda= zeros(nVARS, nVARS);

            for (uint m = 0; m < nTaskParameters ; m++)
            {
                // Projecting Zmu of state "i" at taskParameter "m"
                tmpMu= _TPs[m].A * GMMS[m].getMU(i) + _TPs[m].b;

                // Projecting Zsigma of state "i" at taskParameter "m"
                tmpLambda= invAT[m] * GMMS[m].getLAMBDA(i) * invA[m];

                // Accumulate
                if(framePRIORSProd) {
                    accLambda += pow(refPRIORS(i, m),1) * tmpLambda;

                    accMu += pow(refPRIORS(i, m),1) * tmpLambda * tmpMu;
                }
                else{
                    accLambda += tmpLambda;

                    accMu += tmpLambda * tmpMu;
                }
            }

            // Saving the Gaussian distribution for state "i"
            prodGMM->setMU(i,accLambda.i()*accMu);
            prodGMM->setLAMBDA(i,accLambda);
        }
        prodGMM->setPRIORS(this->PRIORS);
    }

    /*
    for (uint i=0;i<nSTATES;i++)
    {
        cout << prodGMM->getMU(i) << endl;
        cout << prodGMM->getSIGMA(i) << endl;
    }
    */

    return prodGMM;
}

void TPDPGMM::saveInFiles(std::string path) {
	TPGMM::saveInFiles(path);
	tpTrans->saveInFiles(path);
	framePRIORS.save(path + "TPGMM_frame_PRIORS.txt",raw_ascii);
	colvec nbPts(1);
	nbPts(0) = nbPoints;
	nbPts.save(path + "TPGMM_nbPoints.txt",raw_ascii);
}
void TPDPGMM::loadFromFiles(std::string path)
{
	TPGMM::loadFromFiles(path);
}
}


