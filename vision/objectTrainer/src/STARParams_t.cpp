#ifndef STARPARAMS_T_CPP
#define STARPARAMS_T_CPP

struct STARParams_t {
	int maxSize;
	int responseThreshold;
	int lineThresholdProjected;
	int lineThresholdBinarized;
	int suppressNonmaxSize;

	STARParams_t() : 
	maxSize(16),
	responseThreshold(30),
	lineThresholdProjected(10),
	lineThresholdBinarized(8),
	suppressNonmaxSize(5) {}

	STARParams_t(int MS, int RT, int LTP, int LTB, int SNM) :
	maxSize(MS),
	responseThreshold(RT),
	lineThresholdProjected(LTP),
	lineThresholdBinarized(LTB),
	suppressNonmaxSize(SNM) {}

};

#endif
