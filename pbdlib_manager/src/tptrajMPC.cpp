/*
 * tptrajMPC.cpp
 *
 *  Created on: 7 Jan 2016
 *      Author: ihavoutis
 */

#include "pbdlib/tptrajMPC.h"

namespace pbdlib {

TpTrajMPC::~TpTrajMPC() {
	// TODO Auto-generated destructor stub
}

colvec&  TpTrajMPC::computeControlCommand(colvec& X) //X is cur pos
{
	// Construct the Su and Sx matrices
	uint nbData = Np;
	uint nbVar = getNumVARS();
	uint nbVarPos = getNumVARSPos();

	mat	Su = zeros( nbVar*nbData, nbVarPos*(nbData-1) );
	mat I(nbVar,nbVar,fill::eye);
	mat Sx = kron(ones(nbData,1), I);

	mat M = getInputDynamics();

	for (uint n=1; n==nbData; n++) {
//		%Build Sx matrix
		uvec id1 = linspace<uvec>(n*nbVar+1, nbData*nbVar);

		Sx.rows(id1) = Sx.rows(id1) * getSystemDynamics();
//		Sx(id1,:) = Sx(id1,:) * A;
//		%Build Su matrix
		id1 = linspace<uvec>(n*nbVar+1, (n+1)*nbVar);
		uvec id2 = linspace<uvec>(1, n*nbVarPos);

		Su.elem(id1,id2) = M;
//		Su(id1,id2) = M;
//		M = [A*M(:,0:model.nbVarPos), M];
		M = join_horiz( getSystemDynamics() * M.cols(0,nbVarPos-1), M);
}

	uint nbFrames = tp->getNumTASKPARAMETERS();
	//GMM projection, see Eq. (5)
	vector<GMM_Model> gmm_in_frames;
		for (uint m = 0; m < nbFrames; ++m) {
			gmm_in_frames.push_back(tpdpgmm->getGMMS(m));
		}

	// Update hidden markov model
	hsmm->stepForwardVariable(X, in);

	// Create State Sequence:
	// Alpha Predictions
	hsmm->predictForwardVariable(AlphaPred);

	// Calculate the largest alpha values
	maxAlpha = max(AlphaPred,0);

	// Find corresponding corresponding indices (i.e. argmax())
	for (uint i = 0;i<myMPC->getNp();i++)
	{
		tmpAlphaCol = AlphaPred.col(i);
		tmpMax = ones(hsmm->getNumSTATES(),1)*maxAlpha(i);
		q(i)= accu(find(tmpAlphaCol==tmpMax, 1,"first"));
	}

//cout << q.t() << endl;

	vector<mat> MuQ, SigmaQ;
	//Build a reference trajectory for each frame, see Eq. (27)
		mat invSigmaQ = zeros(nbVar*nbData, nbVar*nbData);
		for (uint m=0; m<nbFrames; m++){
			mat mtemp;
			for (int j = 0; j < myMPC->getNp(); ++j) {
				mtemp = join_vert(mtemp, gmm_in_frames[m].getMU(q(j)).col(0));
			}
			MuQ.push_back(mtemp);
//			MuQ.push_back( reshape(gmm_in_frames[m].getMU(0).cols(q),
//					nbVar*nbData, 1 ) );
			mat ctemp;
			for (int j = 0; j < myMPC->getNp(); ++j) {
				ctemp = join_horiz(ctemp, gmm_in_frames[m].getSIGMA(q(j)));
			}
			mat tA,tB,tC;
			tA = kron(ones(nbData,1),I.eye(nbVar,nbVar));
			tB = ctemp;
			tC = kron(I.eye(nbData,nbData), ones(nbVar,nbVar));
			SigmaQ.push_back( (tA * tB) % tC );
//			SigmaQ.push_back( (kron(ones(nbData,1),I.eye(nbVar,nbVar)) * ctemp) %
//					kron(I.eye(nbData,nbData), ones(nbVar,nbVar)) );
			invSigmaQ = invSigmaQ + inv(SigmaQ.back());
		}

//		R = eye(nbVarPos*Np,nbVarPos*Np)*(alpha*alpha);

//		%Recompute R
		R = I.eye(nbVarPos,nbVarPos) * alpha;
		R = kron(eye(nbData-1,nbData-1),R);

		//%Batch LQR (unconstrained linear MPC), see Eq. (37)
		mat SuInvSigmaQ = Su.t() * invSigmaQ;
		mat Rq = SuInvSigmaQ * Su + R;
		//X = [s(1).Data0(:,1) + randn(model.nbVarPos,1)*0E0; zeros(model.nbVarPos,1)];
	 	mat rq = zeros(nbVar*nbData,1);
		for (uint m = 0; m<nbFrames; m++) {
			rq = rq + solve(SigmaQ[m], MuQ[m] - Sx*X);
		}
		rq = Su.t() * rq;
	 	U = solve(Rq, rq); //%can also be computed with u = lscov(Rq, rq);
	 	U = U.rows(0,3);
		return U;
		//r(n).Data = reshape(Sx*X+Su*u, model.nbVar, nbData);

	//
//	// Update update Reference Q and Muq:
//	updateReference(q);
//
//	// Compute control command:
//	return myMPC->computeControlCommand(X,this->Muq,this->Q,this->R);
//	return TrajMPC::computeControlCommand(X);
}

}
