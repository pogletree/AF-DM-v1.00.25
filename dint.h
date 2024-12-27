//
// Header for for the decimator / interpolator file
//

#ifndef __DINT_H__
#define __DINT_H__

void dint_decimate( short *in, int inlen, short *out9, int &len9 );
void interpolate_9600( float *in, float *out, int length );

#endif
