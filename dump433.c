/* dump433, an RF temp sensor decoder for RTLSDR devices.
 *
 * Copyright (C) 2013 by Nicolas Sauzede <nsauzede@laposte.net>
 *
 * All rights reserved.
 *
 * This source code is covered by the GPLv3 license
 *
 * Heavily inspired by dump1090 :
 * Mode1090, a Mode S messages decoder for RTLSDR devices.
 *
 * Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
 */
 
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <inttypes.h>

#include "rtl-sdr.h"

#define dprintf(...) do{}while(0)

//#define MODES_DEFAULT_RATE         2000000
#define MODES_DEFAULT_RATE         250000
#define MODES_DEFAULT_FREQ         434000000
#define MODES_AUTO_GAIN            -100         /* Use automatic gain. */
#define MODES_MAX_GAIN             999999       /* Use max available gain. */

#define MODES_ASYNC_BUF_NUMBER     12
#define MODES_DATA_LEN             (16*16384)   /* 256k */
#define MODES_PREAMBLE_US 8       /* microseconds */
#define MODES_LONG_MSG_BITS 112
#define MODES_SHORT_MSG_BITS 56
#define MODES_FULL_LEN (MODES_PREAMBLE_US+MODES_LONG_MSG_BITS)

typedef struct ctx {
	rtlsdr_dev_t *dev;
	int exit;
	
	int data_ready;
    uint32_t data_len;              /* Buffer length. */
	void *data;
    uint16_t *magnitude;            /* Magnitude vector */
    uint16_t *maglut;               /* I/Q -> Magnitude lookup table. */

	pthread_cond_t data_cond;
	pthread_mutex_t data_mutex;
} ctx_t;

/* We use a thread reading data in background, while the main thread
 * handles decoding and visualization of data to the user.
 *
 * The reading thread calls the RTLSDR API to read data asynchronously, and
 * uses a callback to populate the data buffer.
 * A Mutex is used to avoid races with the decoding thread. */
void rtlsdrCallback(unsigned char *buf, uint32_t len, void *_ctx) {
    ctx_t *ctx = (ctx_t *)_ctx;

	dprintf( "%s: len=%" PRIu32 "\n", __func__, len);
    pthread_mutex_lock(&ctx->data_mutex);
	dprintf( "%s: locked\n", __func__);
    if (len > MODES_DATA_LEN) len = MODES_DATA_LEN;
    /* Move the last part of the previous buffer, that was not processed,
     * on the start of the new buffer. */
    memcpy(ctx->data, ctx->data+MODES_DATA_LEN, (MODES_FULL_LEN-1)*4);
    /* Read the new data. */
    memcpy(ctx->data+(MODES_FULL_LEN-1)*4, buf, len);
	dprintf( "%s: buf=%" PRIx8 " %" PRIx8 "\n", __func__, buf[0], buf[1]);
    ctx->data_ready = 1;
    /* Signal to the other thread that new data is ready */
    pthread_cond_signal(&ctx->data_cond);
	dprintf( "%s: unlocking\n", __func__);
	//getchar();
    pthread_mutex_unlock(&ctx->data_mutex);
}

/* We read data using a thread, so the main thread only handles decoding
 * without caring about data acquisition. */
void *readerThreadEntryPoint(void *arg) {
    ctx_t *ctx = (ctx_t *)arg;
	rtlsdr_read_async(ctx->dev, rtlsdrCallback, arg, MODES_ASYNC_BUF_NUMBER, MODES_DATA_LEN);
    return NULL;
}

/* Turn I/Q samples pointed by Modes.data into the magnitude vector
 * pointed by Modes.magnitude. */
void computeMagnitudeVector( ctx_t *ctx) {
    uint16_t *m = ctx->magnitude;
    unsigned char *p = ctx->data;
    uint32_t j;

    /* Compute the magnitudo vector. It's just SQRT(I^2 + Q^2), but
     * we rescale to the 0-255 range to exploit the full resolution. */
    for (j = 0; j < ctx->data_len; j += 2) {
        int i = p[j]-127;
        int q = p[j+1]-127;

        if (i < 0) i = -i;
        if (q < 0) q = -q;
        m[j/2] = ctx->maglut[i*129+q];
    }
}

/*
Lacrosse TX :
433MHz, AM, OOK
On=signal 1
Off=signal 0
Bit0=large 1 and fix 0
Bit1=short 1 and fix 0

large=2300us : 1300us 1 and 1000us 0
short=1600us : 500us 1 and 1100us 0

silence : > 9000us=225
samplerate : 0.25MSps : 250kSps => 40us
large : 32.5 + 25 = 57.5
short : 12.5 + 27.5 = 40

noise : 72c (1836)
on : 1009 (4105)
off : 0

*/

/* Detect a frame inside the magnitude buffer pointed by 'm' and of
 * size 'mlen' bytes. Every detected frame is converted into a
 * stream of bits and passed to the function to display it. */
void detectFrame(uint16_t *m, uint32_t mlen) {
	int j;
	/* The frame preamble is made of impulses of 1300 microseconds at
     * the following time offsets:
     *
     * 0   - 1300 usec: first impulse. 1300 us
     * 2400 - 3650 usec: second impulse. 1250 us
     * 3.5 - 4   usec: third impulse.
     * 4.5 - 5   usec: last impulse.
     * 
     * Since we are sampling at 2 Mhz every sample in our magnitude vector
     * is 0.5 usec, so the preamble will look like this, assuming there is
     * an impulse at offset 0 in the array:
     *
     * 0   -----------------
     * 1   -
     * 2   ------------------
     * 3   --
     * 4   -
     * 5   --
     * 6   -
     * 7   ------------------
     * 8   --
     * 9   -------------------
	 */
	int nsilence = 0;
	int nsilences = 0;
	int state = 0;
	int non = 0;
	int nons = 0;
	int noff = 0;
	int noffs = 0;
	int bit0 = 0;
#define NSILENCE_MIN 225
#define NON_MIN 30
#define NOFF_MIN 12
#define SILENCE_TOL 200
#define SILENCE_MID 1000
#define SILENCE_MIN (SILENCE_MID+SILENCE_TOL)
#define SILENCE_MAX (SILENCE_MID-SILENCE_TOL)
#define ON_TOL 0
#define ON_MID 30000
#define ON_MIN (ON_MID+ON_TOL)
#define OFF_TOL 0
#define OFF_MID 3000
#define OFF_MAX (OFF_MID-OFF_TOL)
	for (j = 0; j < mlen - MODES_FULL_LEN*2; j++) {
		if (state == 0){
			if ((m[j] <= SILENCE_MAX) /*&& (m[j] >= SILENCE_MIN)*/){
				nsilence++;
			}
			else{
				if (nsilence > nsilences)
					nsilences = nsilence;
				if (m[j] >= ON_MIN){
					if (nsilence > NSILENCE_MIN){
						non = 0;
						state = 1;
					}
				}
				if (m[j] >= SILENCE_MAX)
					nsilence = 0;
			}
		}
		if (state == 1){
			if (m[j] > ON_MIN){
				non++;
				nons++;
			}
			else if (non > NON_MIN){
				noff = 0;
				state = 2;
			}
		}
		if (state == 2){
			if (m[j] < OFF_MAX){
				noff++;
				noffs++;
			}
			else if (noff > NOFF_MIN){
				bit0++;
				state = 0;
				nsilence = 0;
			}
		}
	}
	printf( "%s: bit0=%d nsilences=%d nons=%d noffs=%d m=%" PRIx16 "\n", __func__, bit0, nsilences, nons, noffs, m[0]);
	if (bit0 > 0 || nsilences > 0){
		//getchar();
	}
}

/* This function is called a few times every second by main in order to
 * perform tasks we need to do continuously, like accepting new clients
 * from the net, refreshing the screen in interactive mode, and so forth. */
void backgroundTasks( ctx_t *ctx) {
	dprintf( "%s: hello\n", __func__);
}

int main()
{
    int j;
    int q;
	ctx_t ctx;
	memset( &ctx, 0, sizeof( ctx));
	
    pthread_mutex_init(&ctx.data_mutex,NULL);
    pthread_cond_init(&ctx.data_cond,NULL);
    /* We add a full message minus a final bit to the length, so that we
     * can carry the remaining part of the buffer that we can't process
     * in the message detection loop, back at the start of the next data
     * to process. This way we are able to also detect messages crossing
     * two reads. */
    ctx.data_len = MODES_DATA_LEN + (MODES_FULL_LEN-1)*4;
    ctx.data_ready = 0;
	ctx.data = malloc( ctx.data_len);
	ctx.magnitude = malloc( ctx.data_len*2);
    memset(ctx.data,127,ctx.data_len);

    /* Populate the I/Q -> Magnitude lookup table. It is used because
     * sqrt or round may be expensive and may vary a lot depending on
     * the libc used.
     *
     * We scale to 0-255 range multiplying by 1.4 in order to ensure that
     * every different I/Q pair will result in a different magnitude value,
     * not losing any resolution. */
    ctx.maglut = malloc(129*129*2);
    for (j = 0; j <= 128; j++) {
        for (q = 0; q <= 128; q++) {
            ctx.maglut[j*129+q] = round(sqrt(j*j+q*q)*360);
        }
    }
	
    pthread_t reader_thread;
	int dev_index = 0;
    int gain = MODES_MAX_GAIN;
    int enable_agc = 0;
    int freq = MODES_DEFAULT_FREQ;
	
    int device_count;
    int ppm_error = 0;
    char vendor[256], product[256], serial[256];

	printf( "hello dump433\n");
    device_count = rtlsdr_get_device_count();
    if (!device_count) {
        fprintf(stderr, "No supported RTLSDR devices found.\n");
        exit(1);
    }

    fprintf(stderr, "Found %d device(s):\n", device_count);
    for (j = 0; j < device_count; j++) {
        rtlsdr_get_device_usb_strings(j, vendor, product, serial);
        fprintf(stderr, "%d: %s, %s, SN: %s %s\n", j, vendor, product, serial,
            (j == dev_index) ? "(currently selected)" : "");
    }

    if (rtlsdr_open(&ctx.dev, dev_index) < 0) {
        fprintf(stderr, "Error opening the RTLSDR device: %s\n",
            strerror(errno));
        exit(1);
    }

	/* Set gain, frequency, sample rate, and reset the device. */
	rtlsdr_set_tuner_gain_mode(ctx.dev, (gain == MODES_AUTO_GAIN) ? 0 : 1);
	if (gain != MODES_AUTO_GAIN) {
		if (gain == MODES_MAX_GAIN) {
            /* Find the maximum gain available. */
            int numgains;
            int gains[100];

            numgains = rtlsdr_get_tuner_gains(ctx.dev, gains);
            gain = gains[numgains-1];
            fprintf(stderr, "Max available gain is: %.2f\n", gain/10.0);
        }
        rtlsdr_set_tuner_gain( ctx.dev, gain);
        fprintf(stderr, "Setting gain to: %.2f\n", gain/10.0);
    } else {
        fprintf(stderr, "Using automatic gain control.\n");
    }
    rtlsdr_set_freq_correction( ctx.dev, ppm_error);
	if (enable_agc)
		rtlsdr_set_agc_mode(ctx.dev, 1);
	printf( "setting center freq to %d Hz\n", freq);
    rtlsdr_set_center_freq(ctx.dev, freq);
    rtlsdr_set_sample_rate(ctx.dev, MODES_DEFAULT_RATE);
    rtlsdr_reset_buffer(ctx.dev);
    fprintf(stderr, "Gain reported by device: %.2f\n",
	rtlsdr_get_tuner_gain(ctx.dev)/10.0);
	
    /* Create the thread that will read the data from the device. */
    pthread_create(&reader_thread, NULL, readerThreadEntryPoint, &ctx);

    pthread_mutex_lock(&ctx.data_mutex);
	dprintf( "%s: locked\n", __func__);
    while(1) {
        if (!ctx.data_ready) {
            pthread_cond_wait(&ctx.data_cond,&ctx.data_mutex);
            continue;
        }
        computeMagnitudeVector( &ctx);

        /* Signal to the other thread that we processed the available data
         * and we want more (useful for --ifile). */
        ctx.data_ready = 0;
        pthread_cond_signal(&ctx.data_cond);

        /* Process data after releasing the lock, so that the capturing
         * thread can read data while we perform computationally expensive
         * stuff * at the same time. (This should only be useful with very
         * slow processors). */
		dprintf( "%s: unlocking\n", __func__);
        pthread_mutex_unlock(&ctx.data_mutex);
        detectFrame(ctx.magnitude, ctx.data_len/2);
        backgroundTasks( &ctx);
        pthread_mutex_lock(&ctx.data_mutex);
		dprintf( "%s: locked\n", __func__);
        if (ctx.exit) break;
    }
    rtlsdr_close(ctx.dev);

	return 0;
}
