/*
 * relativeUnifreinject.c
 *
 *  Created on: Oct 31, 2014
 *      Author: chenxiaoyu
 */
#include "relativeUnifreinject.h"
#include "Tools.h"
#include "randNormal.h"
#include <math.h>

void relativeUnifreinject(particleCloud* XBar, float bearing, float orientation, float range){

    float rawThe[M];
    randNormal(rawThe, M);
    float rawRho[M];
    randNormal(rawRho, M);
    float rawOri[M];
    randNormal(rawOri, M);
	for (int i = 0; i < M; i ++){
		rawThe[i] = rawThe[i] * rbearingSig + bearing + rbearingMu;
		rawRho[i] = rawRho[i] * rrangeSig + rrangeMu + range;
		rawOri[i] = rawOri[i] * rorientationSig + rorientationMu + bearing + M_PI - orientation;
	}
    
    cartCoordinates xAndy;
    cartCoordinates* toxAndy = &xAndy;
    
	normalizeAngle(rawThe);

	pol2cart(toxAndy, rawThe, rawRho);
    
	normalizeAngle(rawOri);

    for (int i = 0; i < M; i++){
        XBar -> partX[i] = xAndy.x[i];
        XBar -> partY[i] = xAndy.y[i];
        XBar -> partHeading[i] = rawOri[i];
    }
}

