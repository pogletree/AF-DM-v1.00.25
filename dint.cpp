//
// This file does decimation and interpolation of the sample rate.
// Using a sample rate of 48000 it decimates to 9600.
//
// On the transmit side it does the same, interpolating from 9600 to
// 48000.
//
// For this to work the input block size has to be a multiple of 30;
// Anti aliasing is done by the radios filters.
//

// To do calibration Charles comments on adding the offset are:
/*

From: Charles Brain <chbrain@btinternet.com>
To: NNN0WWL <nnn0wwl@marsale.org>
Subject: Re: 1st radio test bed results 

Steve,

I have had this recommended to me.
http://www.mega-nerd.com/SRC/

48K is the basic rate for AC'97
9600 is just 48000 divided by 5, being an integer
relationship the sample rate should be precise.
If they can't even get that right then it is a pretty
crap soundcard.

Why not use 48000 and decimate by 5 to get 9600
in your code ?

Well you could try something crude like this.

double acc;
double adj = 0.998; // or what ever

adjust_sample_rate( sample )
{
    acc += adj;
    while( acc >= 1.0 )
    {
        output_sample( sample );
        acc -= 1.0;
    }
}

It will add distortion to the waveform and is best done at the raw
48K rate but with all the decimation afterwards and filtering it might
be all right.

To prevent the distortion you will probably have to do some kind of sample
interpolation which starts to get complicated.

- Charles


*/

#include <stdio.h>
#include <direct.h>



// Filter sample rate 48000, cutoff 4 Khz
// Filter length must be a multiple of the 
// Interpolation factor (5).

#define IFILTER_LENGTH 55

static float icoeffs[IFILTER_LENGTH] = {
	 0.000941f,
	 0.000852f,
	 0.000534f,
	-0.000072f,
	-0.000965f,
	-0.001999f,
	-0.002839f,
	-0.003012f,
	-0.002066f,
	 0.000206f,
	 0.003551f,
	 0.007157f,
	 0.009752f,
	 0.009923f,
	 0.006604f,
	-0.000413f,
	-0.010100f,
	-0.020129f,
	-0.027208f,
	-0.027776f,
	-0.018944f,
	 0.000594f,
	 0.029931f,
	 0.065809f,
	 0.103079f,
	 0.135689f,
	 0.157963f,
	 0.165873f,
	 0.157963f,
	 0.135689f,
	 0.103079f,
	 0.065809f,
	 0.029931f,
	 0.000594f,
	-0.018944f,
	-0.027776f,
	-0.027208f,
	-0.020129f,
	-0.010100f,
	-0.000413f,
	 0.006604f,
	 0.009923f,
	 0.009752f,
	 0.007157f,
	 0.003551f,
	 0.000206f,
	-0.002066f,
	-0.003012f,
	-0.002839f,
	-0.001999f,
	-0.000965f,
	-0.000072f,
	 0.000534f,
	 0.000852f,
	 0.000941f
};

static float outdata[IFILTER_LENGTH/5];


// RX starts here ------------------------------------------------------------------

//
// Sample at 48000 and decimate to 9600 samples/sec. for decoding data
// This relies on the radios SSB filter for anti aliasing.
//
// unsigned short *in are RX samples at 48000
//
#include <QByteArray>
#include <QDebug>

void dint_decimate( short *in, int inlen, short *out9, int &len9 )
{
    //qDebug()<<"dint in = "<<in[0]<<in[1]<<in[2]<<in[3]<<in[4]<<in[5]<<in[6];

    int i;

	len9 = 0;

	for( i = 0; i < inlen; i++ )
	{
		if( i%5 == 0 )
		{
            //qDebug()<<"dint mod 5 = 0 "<<in[i]<<in[i]+32768;
            //unsigned short u = in[i] + 32768;
            out9[len9++] = in[i];
        }
	}


    //qDebug()<<"dint out = "<<out9[0]<<out9[1]<<out9[2];
}


// TX starts here ------------------------------------------------------------------

//
// 4 Khz LPF for 9600 -> 48000 interpolation on TX
//
static float ifilter9600( int offset )
{
	float out;

	out = outdata[0]*icoeffs[offset];

	for( int i = 1; i < (IFILTER_LENGTH/5); i++ )
	{
		out += outdata[i]*icoeffs[(i*5)+offset];
	}
	
	// Make sure no coefficient is above .999999
	if( out > 0.999999)
	{
		out = out*0.1f;
	}

	if( out > 0.999999)
	{
		out = out-1.0f;
	}

	
/*
	//if( out > 0.999999)// Save any coefficient above .999999
	if( out > 0.000000000001) // Basically save all
	{

		FILE *out_fp; // File pointer of logging file
		static char outFileName[255];
		static char *CurrentPath;
		static char *outlogSubDirectory;
		int curdrive;

		outlogSubDirectory = "\\outlog\\";

		// Fix to allow for support on any system drive
		curdrive = _getdrive();
		CurrentPath = _getdcwd( curdrive, NULL, 0 );

		strcpy(outFileName,CurrentPath);
		strcat(outFileName,outlogSubDirectory);
		strcat(outFileName,"test");
		strcat(outFileName,".out");

		//MessageBox (NULL, outFileName, "test.out Path", MB_OK);

		out_fp = fopen(outFileName,"a");

		if(out_fp != NULL)
		{
			//fputs(LPCTSTR(out),out_fp);
			fprintf (out_fp,"%f\n",out);

		}
		//fclose(out_fp);
	}

*/

	return out;

}
//
// Put new sample into the filter buffer for TX.
//
static void update_output_buffer( float in )
{
	int i;

	for( i = 0; i < (IFILTER_LENGTH/5)-1; i++ )
	{
		outdata[i] = outdata[i+1];
	}
	outdata[i] = in*5;
	
}
//
// Interpolate from 9600 to 48000 for transmitting data
//
void   interpolate_9600( float *in, float *out, int length )
{
	int i,j;

	for( i = 0, j = 0; i < length; i++ )
	{
		update_output_buffer( in[i] );
		out[j++] = ifilter9600( 0 );
		out[j++] = ifilter9600( 1 );
		out[j++] = ifilter9600( 2 );
		out[j++] = ifilter9600( 3 );
		out[j++] = ifilter9600( 4 );
	}
}
