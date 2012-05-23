/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <errno.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>

#include <string.h>
#include <inttypes.h>

// for timer
#include <time.h>
#include <sched.h>
#include <sys/io.h>

#include "ach.h"


typedef double x_t[3]; /* state / feedback: time, position, velocity */
typedef double u_t[1]; /* input message: velocity */

/* channels */
ach_channel_t chan_control;
ach_channel_t chan_feedback;

double now() {
    struct timespec t;
    clock_gettime( CLOCK_MONOTONIC, &t );
    return (double)(t.tv_sec) + (double)t.tv_nsec / 1e9 ;
}

/* simple integrator, x = dt * dx */
void robot(void) {
    int r = ach_open(&chan_feedback, "feedback", NULL);
    assert(ACH_OK == r);
    r = ach_open(&chan_control, "control", NULL);
    assert(ACH_OK == r);
    x_t X = {now(),0,0};
    while(1) {
        u_t U;
        size_t fs;
        r = ach_get( &chan_control, U, sizeof(U), &fs, NULL, ACH_O_WAIT|ACH_O_LAST );
        assert( (ACH_OK==r || ACH_MISSED_FRAME==r) && sizeof(U) == fs );
        double tm = now();
        X[2] = U[0];               /*  dx = u       */
        X[1] = (tm - X[0]) * X[2]; /*  x = dt * dx  */
        X[0] = tm;
        ach_put(&chan_feedback, X, sizeof(X));
    }
    exit(0);
}

/* print samples periodically */
void periodic_logger(void) {
    int r = ach_open(&chan_feedback, "feedback", NULL);
    assert(ACH_OK == r);
    while(1) {
        x_t X;
        size_t fs;
        r = ach_get( &chan_feedback, X, sizeof(X), &fs, NULL, ACH_O_WAIT|ACH_O_LAST );
        assert( (ACH_OK==r || ACH_MISSED_FRAME==r) && sizeof(X) == fs );
        printf("dan - %f\t%f\t%f\n", X[0], X[1], X[2]);
        usleep((int) (1e6 * 0.1)); /* 10 Hertz */
    }
    exit(0);
}

/* log all samples to a file */
void full_logger(void) {
    int r = ach_open(&chan_feedback, "feedback", NULL);
    assert(ACH_OK == r);
    FILE *fp = fopen("ach-example.dat", "w");
    assert(fp);
    while(1) {
        x_t X;
        size_t fs;
        r = ach_get( &chan_feedback, X, sizeof(X), &fs, NULL, ACH_O_WAIT );
        assert( (ACH_OK==r || ACH_MISSED_FRAME==r) && sizeof(X) == fs );
        fprintf(fp,"dan - %f\t%f\t%f\n", X[0], X[1], X[2]);
    }
    fclose(fp);
    exit(0);
}

/* sinusoidal input */
void controller(void) {
    int r = ach_open(&chan_control, "control", NULL);
    assert(ACH_OK == r);
    while(1){
        double tm = now();
        u_t U = {sin(tm)};
        ach_put(&chan_control, U, sizeof(U));
        usleep((int)(1e6 * 1e-3)); /* kilohertz */
    }
    exit(0);
}

// Dan input
void dan(void) {
    	int r = ach_open(&chan_feedback, "feedback", NULL);
	struct timespec t;

	









	assert(ACH_OK == r);
	while(1){
		printf("test");
		usleep((int)(1e6 * 1e-1)); // KHZ
	}
}

int main(int argc, char **argv) {
    (void) argc; (void)argv;
    int r;
    /* create channels */
    r = ach_unlink("control");               /* delete first */
    assert( ACH_OK == r || ACH_ENOENT == r);
    r = ach_unlink("feedback");              /* delete first */
    assert( ACH_OK == r || ACH_ENOENT == r);
    r = ach_unlink("dan");
    assert( ACH_OK == r || ACH_ENOENT == r);


    r = ach_create("control", 10ul, 256ul, NULL );
    assert(ACH_OK == r);
    r = ach_create("feedback", 10ul, 256ul, NULL );
    assert(ACH_OK == r);
    r = ach_create("dan", 10ul, 256ul, NULL );
    assert(ACH_OK == r);




    /* fork processes */
    int pid_ctrl = fork();
    assert(pid_ctrl >= 0);
    if(!pid_ctrl) controller();

    int pid_bot = fork();
    assert(pid_bot >= 0);
    if(!pid_bot) robot();

    int pid_periodic = fork();
    assert(pid_periodic >= 0);
    if(!pid_periodic) periodic_logger();

    int pid_full = fork();
    assert(pid_full >= 0);
    if(!pid_full) full_logger();

    int pid_dan = fork();
    assert(pid_dan >=0);
    if(!pid_dan) dan();

    /* wait for a signal */
    pause();
    return 0;
}
