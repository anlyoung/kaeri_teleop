/*
 * tptrajMPC.h
 *
 *  Created on: 7 Jan 2016
 *      Author: ihavoutis
 */

#ifndef RLI_PBDLIB_SANDBOX_SRC_TPTRAJMPC_H_
#define RLI_PBDLIB_SANDBOX_SRC_TPTRAJMPC_H_

#include "armadillo"
#include "pbdlib/trajMPC.h"
#include "pbdlib/tpdpgmm.h"
#include "pbdlib/taskparameters.h"

namespace pbdlib {

class TpTrajMPC: public TrajMPC {

public:

	TpTrajMPC(TPDPGMM* _tpdpgmm, TaskParameters* _tp,
			HSMM* _hsmm,
			double _dt, uint _nbVarPos, uint _nbDeriv,
			uint _Np, uint _Nc, double _alpha)
	:TrajMPC(_hsmm, _dt, _nbVarPos, _nbDeriv, _Np, _Nc, _alpha)
	{
		Np = _Np; // prediction horizon
		tpdpgmm = _tpdpgmm;
		tp = _tp;
		alpha = _alpha;
	}

	virtual ~TpTrajMPC();

	colvec& computeControlCommand(colvec& X);

private:
	uint Np;
	TPDPGMM* tpdpgmm;
	TaskParameters* tp;
	double alpha;
};
}
#endif /* RLI_PBDLIB_SANDBOX_SRC_TPTRAJMPC_H_ */
