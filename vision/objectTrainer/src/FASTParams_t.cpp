#ifndef FASTPARAMS_T_CPP
#define FASTPARAMS_T_CPP

struct FASTParams_t {
	int threshold;
	bool nonmaxSuppression;
	
	FASTParams_t () : threshold(1), nonmaxSuppression(true) {}

	FASTParams_t (int t, bool s) : threshold(t), nonmaxSuppression(s) {}
};

#endif
