
#ifndef SURFPARAMS_T_CPP
#define SURFPARAMS_T_CPP

struct SURFParams_t {
	double hessianThreshold; // Threshold for hessian keypoint detector used in SURF.
	int nOctaves; // Number of pyramid octaves the keypoint detector will use. (DEFAULT = 4)
	int nOctaveLayers; // Number of octave layers within each octave. (DEFAULT = 2)
	bool extended;// Extended descriptor flag (true - use extended 128-element descriptors; false - use 64-element descriptors). (DEFAULT = TRUE)
	bool upright; // Up-right or rotated features flag (true - do not compute orientation of features; false - compute orientation). (DEFAULT = FALSE)

	SURFParams_t(double HESSIANTHRESHOLD) : 
	hessianThreshold(HESSIANTHRESHOLD),
	nOctaves(4),
	nOctaveLayers(2),
	extended(true),
	upright(false) {}

	SURFParams_t(double HESSIANTHRESHOLD, int NOCTAVES, int NOCTAVELAYERS, bool EXTENDED, bool UPRIGHT) : 
	hessianThreshold(HESSIANTHRESHOLD),
	nOctaves(NOCTAVES),
	nOctaveLayers(NOCTAVELAYERS),
	extended(EXTENDED),
	upright(UPRIGHT) {}

};

#endif
