/********************************************************************
* Description: genserkins.c
*   Kinematics for a generalised serial kinematics machine
*
*   Derived from a work by Fred Proctor,
*   changed to work with emc2 and HAL
*
* Adapting Author: Alex Joni
* License: GPL Version 2
* System: Linux
*
*******************************************************************

  These are the forward and inverse kinematic functions for a general
  serial-link manipulator. Thanks to Herman Bruyninckx and John
  Hallam at http://www.roble.info/ for this.

  The functions are general enough to be configured for any serial
  configuration.
  The kinematics use Denavit-Hartenberg definition for the joint and
  links. The DH definitions are the ones used by John J Craig in
  "Introduction to Robotics: Mechanics and Control"
  The parameters for the manipulator are defined by hal pins.
  Currently the type of the joints is hardcoded to ANGULAR, although
  the kins support both ANGULAR and LINEAR axes.

  TODO:
    * make number of joints a loadtime parameter
    * add HAL pins for all settable parameters, including joint type: ANGULAR / LINEAR
    * add HAL pins for debug data (num_iterations)
    * add HAL pins for ULAPI compiled version
*/

#include "uarmkins.h"		/* these decls */

#include "rtapi_math.h"
#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "kinematics.h"

#ifdef RTAPI
#include "rtapi.h"
#include "rtapi_app.h"
#endif

#include "hal.h"
struct haldata {
    hal_float_t *d[UARM_LINKS];
    uarm_struct *kins;
    go_pose *pos;		// used in various functions, we malloc it
				// only once in rtapi_app_main
} *haldata = 0;

double j[UARM_JOINTS];


#define D(i) (*(haldata->d[i]))

#define KINS_PTR (haldata->kins)

//#define HAL_W  (haldata->w)

int uarm_kin_init(void) {

    return GO_RESULT_OK;
}

go_real tt[4] = {0};
/* main function called by emc2 for forward Kins */
int kinematicsForward(const double *joint,
                      EmcPose * world,
                      const KINEMATICS_FORWARD_FLAGS * fflags,
		      KINEMATICS_INVERSE_FLAGS * iflags) {


    go_real jcopy[UARM_JOINTS]; // will hold the radian conversion of joints
    go_real Px, Py, Pz, Pt;
    go_real s1, c1, s2, c2, s3, c3, s4, c4;
    int i, changed=0;

    for (i=0; i< UARM_JOINTS; i++)  {
	// FIXME - debug hack
    	if (!GO_ROT_CLOSE(j[i],joint[i])) changed = 1;
	// convert to radians to pass to genser_kin_fwd
    	jcopy[i] = joint[i] * PM_PI / 180;
    }

    if (changed) {
    	for (i=0; i< UARM_JOINTS; i++)
    		j[i] = joint[i];
    	//rtapi_print("kinematicsForward(joints: %f %f %f %f %f %f)\n", joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]);
    	}


    sincos(jcopy[0], &s1, &c1);
    sincos(jcopy[1], &s2, &c2);
    sincos(jcopy[2], &s3, &c3);
    sincos(jcopy[5] + jcopy[0], &s4, &c4);
	Pt = D(1) + c2*D(2) + c3*D(3);
	Px = Pt * c1;
	Py = Pt * s1;
	Pz = s2*D(2) + s3*D(3) + D(0);
    world->tran.x = Px + D(4) * c1 + D(6) * c4;
    world->tran.y = Py + D(4) * s1 + D(6) * s4;
    world->tran.z = Pz - D(5);
    world->c = joint[5] + joint[0];
    if (changed) {
     // rtapi_print("kinematicsForward(joints: %f %f %f %f %f %f)\n", joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]);
     // rtapi_print("kinematicsForward(world: %f %f %f %f )\n", world->tran.x, world->tran.y, world->tran.z, world->c);
	  //rtapi_print("kinematicsForward(wrist: %f %f %f %f %f %f)\n\n\n", Pt, Px, Py, Pz, c2, c3);
	//rtapi_print("kinematicsForward(D: %f %f %f %f %f %f %f)\n", D(0), D(1), D(2), D(3), D(4), D(5), D(6) );
    }
    return GO_RESULT_OK;
}

int kinematicsInverse(const EmcPose * world,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS * iflags,
		      KINEMATICS_FORWARD_FLAGS * fflags)
{

	go_real Pxy0, Pz0, sqPxy0, sqPz0, Pt0, Pt1, a, b, c, y, x, y0, x0,j4, c4, s4;
	go_real jest[UARM_JOINTS];
//	EmcPose pos;
	int i, changed = 0;
	//rtapi_print("start compute_jinv (world: %f %f %f %f)\n", world->tran.x, world->tran.y, world->tran.z, world->c );
    j4 = world->c * PM_PI / 180;
    sincos(j4, &s4, &c4);
    y = world->tran.y - s4 * D(6);
    x = world->tran.x - c4 * D(6);
	jest[0] = atan2(y, x);
//	if (world->c > 0)
		jest[5] = j4 - jest[0];
//	else
	//	jest[5] = -j4 - jest[0];

	Pxy0 = sqrt(go_sq(y) + go_sq(x)) - D(1) -D(4);
    Pz0 = world->tran.z - D(0) + D(5);
    sqPxy0 = go_sq(Pxy0);
    sqPz0 = go_sq(Pz0);
    Pt0 = sqPxy0 + sqPz0 + go_sq(D(2)) - go_sq(D(3));
    a = 4*(sqPz0 + sqPxy0);
    b = -4 * Pz0 * Pt0;
    c = go_sq(Pt0) - 4*sqPxy0*go_sq(D(2));
    Pt1 = go_sq(b) - 4*a*c;

    if (Pt1 < 0) {
    //	rtapi_print("%f %f %f %f %f %f %f %f %f %f %f %f %f\n", x, y, jest[0]* 180 / PM_PI, jest[5]* 180 / PM_PI,
    //			Pxy0, Pz0,
//				sqPxy0,  sqPz0,
//				Pt0, a, b, c, Pt1);
rtapi_print("ERR kI - compute_jinv (world: %f %f %f %f)\n", world->tran.x, world->tran.y, world->tran.z, world->c );
    	return GO_RESULT_IGNORED;
    }

    y0 = (sqrt(Pt1) - b) / (2*a);
    x0 =  (Pt0 - 2*Pz0*y0 ) / (2 * Pxy0);
    jest[1] = atan2(y0, x0);
    jest[2] = atan2(Pz0 - y0, Pxy0 - x0);

    joints[0] = jest[0] * 180 / PM_PI;
    joints[1] = jest[1] * 180 / PM_PI;
    joints[2] = jest[2] * 180 / PM_PI;
    joints[5] = jest[5] * 180 / PM_PI;


    if (!GO_ROT_CLOSE(tt[0],world->tran.x )) changed = 1;
    if (!GO_ROT_CLOSE(tt[1],world->tran.y )) changed = 1;
    if (!GO_ROT_CLOSE(tt[2],world->tran.z )) changed = 1;
    if (!GO_ROT_CLOSE(tt[3],world->c )) changed = 1;

    if(changed){
    //	rtapi_print("start compute_jinv (world: %lf %lf %lf %lf)\n", world->tran.x, world->tran.y, world->tran.z, world->c );
    	//rtapi_print("%lf %lf\n %lf %lf \n%lf %lf \n%lf %lf \n%lf \n%lf %lf %lf\n %lf \n%lf %lf \n", x, y, jest[0]* 180 / PM_PI, jest[5]* 180 / PM_PI,
    	    	//		Pxy0, Pz0,
    		//			sqPxy0,  sqPz0, Pt0, a, b, c, Pt1, y0, x0);
    //	rtapi_print("kineInverse(joints: %lf %lf %lf %lf %lf %lf %lf)\n", joints[0],joints[1],joints[2],joints[3],joints[4],joints[5], jest[2] * 180 / PM_PI);
    	tt[0] = world->tran.x;
    	tt[1] = world->tran.y;
    	tt[2] = world->tran.z;
    	tt[3] = world->c;

    //	kinematicsForward(joints, &pos, &i, &changed);
    //	rtapi_print("varied compute_jinv (pos: %lf %lf %lf %lf)\n\n\n", pos.tran.x, pos.tran.y, pos.tran.z, pos.c );
    }
return GO_RESULT_OK;
}

int kinematicsHome(EmcPose * world,
    double *joint,
    KINEMATICS_FORWARD_FLAGS * fflags, KINEMATICS_INVERSE_FLAGS * iflags)
{
    /* use joints, set world */
	rtapi_print("kinematicsForward(joints: %f %f %f %f %f %f)\n", joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]);
    return kinematicsForward(joint, world, fflags, iflags);
}

KINEMATICS_TYPE kinematicsType()
{
    return KINEMATICS_BOTH;
}

#ifdef RTAPI

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

int comp_id;

int rtapi_app_main(void)
{
    int res = 0, i;

    comp_id = hal_init("uarmkins");
    if (comp_id < 0)
	return comp_id;

    haldata = hal_malloc(sizeof(struct haldata));
    if (!haldata)
	goto error;

    for (i = 0; i < UARM_LINKS; i++) {
    	if ((res =
    			hal_pin_float_newf(HAL_IO, &(haldata->d[i]), comp_id,
    					"uarmkins.d-%d", i)) < 0)
    				goto error;
        *(haldata->d[i])=0;

    }

    KINS_PTR = hal_malloc(sizeof(uarm_struct));
    haldata->pos = (go_pose *) hal_malloc(sizeof(go_pose));
    if (KINS_PTR == NULL)
	goto error;
    if (haldata->pos == NULL)
	goto error;



    D(0) = DEFAULT_D0;
    D(1) = DEFAULT_D1;
    D(2) = DEFAULT_D2;
    D(3) = DEFAULT_D3;
    D(4) = DEFAULT_D4;
    D(5) = DEFAULT_D5;
    D(6) = DEFAULT_D6;
    hal_ready(comp_id);
    return 0;

  error:
    hal_exit(comp_id);
    return res;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}
#endif

//building for userspace - we'll do a main()
#ifdef ULAPI

#include <stdio.h>
#include <sys/time.h>		/* struct timeval */
#include <stdlib.h>		/* malloc() */
#include <unistd.h>		/* gettimeofday() */

static double timestamp()
{
    struct timeval tp;

    if (0 != gettimeofday(&tp, NULL)) {
	return 0.0;
    }
    return ((double) tp.tv_sec) + ((double) tp.tv_usec) / 1000000.0;
}

int main(int argc, char *argv[])
{
#define BUFFERLEN 256
    char buffer[BUFFERLEN];
    int inverse = 1;
    int jacobian = 0;
    EmcPose pos = { {0.0, 0.0, 0.0}, 0.0, 0.0, 0.0 };
    EmcPose vel = { {0.0, 0.0, 0.0}, 0.0, 0.0, 0.0 };	// will need this for
							// jacobian
    double joints[6] = { 0.0 };
    double jointvels[6] = { 0.0 };
    KINEMATICS_INVERSE_FLAGS iflags = 0;
    KINEMATICS_FORWARD_FLAGS fflags = 0;
    int t;
    int retval = 0, i;
    double start, end;

    // FIXME-AJ: implement ULAPI HAL version of the pins
    haldata = malloc(sizeof(struct haldata));

    KINS_PTR = malloc(sizeof(genser_struct));
    haldata->pos = (go_pose *) malloc(sizeof(go_pose));

    for (i = 0; i < GENSER_MAX_JOINTS ; i++) {
	haldata->a[i] = malloc(sizeof(double));
	haldata->alpha[i] = malloc(sizeof(double));
	haldata->d[i] = malloc(sizeof(double));
    }
    A(0) = DEFAULT_A1;
    A(1) = DEFAULT_A2;
    A(2) = DEFAULT_A3;
    A(3) = DEFAULT_A4;
    A(4) = DEFAULT_A5;
    A(5) = DEFAULT_A6;
    ALPHA(0) = DEFAULT_ALPHA1;
    ALPHA(1) = DEFAULT_ALPHA2;
    ALPHA(2) = DEFAULT_ALPHA3;
    ALPHA(3) = DEFAULT_ALPHA4;
    ALPHA(4) = DEFAULT_ALPHA5;
    ALPHA(5) = DEFAULT_ALPHA6;
    D(0) = DEFAULT_D1;
    D(1) = DEFAULT_D2;
    D(2) = DEFAULT_D3;
    D(3) = DEFAULT_D4;
    D(4) = DEFAULT_D5;
    D(5) = DEFAULT_D6;

    /* syntax is a.out {i|f # # # # # #} */
    if (argc == 8) {
	if (argv[1][0] == 'f') {
	    /* joints passed, so do interations on forward kins for timing */
	    for (t = 0; t < 6; t++) {
		if (1 != sscanf(argv[t + 2], "%lf", &joints[t])) {
		    fprintf(stderr, "bad value: %s\n", argv[t + 2]);
		    return 1;
		}
	    }
	    inverse = 0;
	} else if (argv[1][0] == 'i') {
	    /* world coords passed, so do iterations on inverse kins for
	       timing */
	    if (1 != sscanf(argv[2], "%lf", &pos.tran.x)) {
		fprintf(stderr, "bad value: %s\n", argv[2]);
		return 1;
	    }
	    if (1 != sscanf(argv[3], "%lf", &pos.tran.y)) {
		fprintf(stderr, "bad value: %s\n", argv[3]);
		return 1;
	    }
	    if (1 != sscanf(argv[4], "%lf", &pos.tran.z)) {
		fprintf(stderr, "bad value: %s\n", argv[4]);
		return 1;
	    }
	    if (1 != sscanf(argv[5], "%lf", &pos.a)) {
		fprintf(stderr, "bad value: %s\n", argv[5]);
		return 1;
	    }
	    if (1 != sscanf(argv[6], "%lf", &pos.b)) {
		fprintf(stderr, "bad value: %s\n", argv[6]);
		return 1;
	    }
	    if (1 != sscanf(argv[7], "%lf", &pos.c)) {
		fprintf(stderr, "bad value: %s\n", argv[7]);
		return 1;
	    }
	    inverse = 1;
	} else {
	    fprintf(stderr, "syntax: %s {i|f # # # # # #}\n", argv[0]);
	    return 1;
	}
	/* need an initial estimate for the forward kins, so ask for it */
	if (inverse == 0) {
	    do {
		printf("initial estimate for Cartesian position, xyzrpw: ");
		fflush(stdout);
		if (NULL == fgets(buffer, BUFFERLEN, stdin)) {
		    return 0;
		}
	    } while (6 != sscanf(buffer, "%lf %lf %lf %lf %lf %lf",
		    &pos.tran.x,
		    &pos.tran.y, &pos.tran.z, &pos.a, &pos.b, &pos.c));
	}

	start = timestamp();
	if (inverse) {
	    retval = kinematicsInverse(&pos, joints, &iflags, &fflags);
	    if (0 != retval) {
		printf("inv kins error %d\n", retval);
	    }
	} else {
	    retval = kinematicsForward(joints, &pos, &fflags, &iflags);
	    if (0 != retval) {
		printf("fwd kins error %d\n", retval);
	    }
	}
	end = timestamp();

	printf("calculation time: %f secs\n", (end - start));
	return 0;
    }

    /* end of if args for timestamping */
    /* else we're interactive */
    while (!feof(stdin)) {
	if (inverse) {
	    if (jacobian) {
		printf("jinv> ");
	    } else {
		printf("inv> ");
	    }
	} else {
	    if (jacobian) {
		printf("jfwd> ");
	    } else {
		printf("fwd> ");
	    }
	}
	fflush(stdout);

	if (NULL == fgets(buffer, BUFFERLEN, stdin)) {
	    break;
	}

	if (buffer[0] == 'i') {
	    inverse = 1;
	    continue;
	} else if (buffer[0] == 'f') {
	    inverse = 0;
	    continue;
	} else if (buffer[0] == 'j') {
	    jacobian = !jacobian;
	    continue;
	} else if (buffer[0] == 'q') {
	    break;
	}

	if (inverse) {
	    if (jacobian) {
		if (12 != sscanf(buffer,
			"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
			&pos.tran.x, &pos.tran.y, &pos.tran.z, &pos.a, &pos.b,
			&pos.c, &vel.tran.x, &vel.tran.y, &vel.tran.z, &vel.a,
			&vel.b, &vel.c)) {
		    printf("?\n");
		} else {
//FIXME-AJ
//disabled for now        retval = jacobianInverse(&pos, &vel, joints, jointvels);
		    printf("%f %f %f %f %f %f\n",
			jointvels[0],
			jointvels[1],
			jointvels[2],
			jointvels[3], jointvels[4], jointvels[5]);
		    if (0 != retval) {
			printf("inv Jacobian error %d\n", retval);
		    } else {
//FIXME-AJ
//disabled for now          retval = jacobianForward(joints, jointvels, &pos, &vel);
			printf("%f %f %f %f %f %f\n",
			    vel.tran.x,
			    vel.tran.y, vel.tran.z, vel.a, vel.b, vel.c);
			if (0 != retval) {
			    printf("fwd kins error %d\n", retval);
			}
		    }
		}
	    } else {
		if (6 != sscanf(buffer, "%lf %lf %lf %lf %lf %lf",
			&pos.tran.x,
			&pos.tran.y, &pos.tran.z, &pos.a, &pos.b, &pos.c)) {
		    printf("?\n");
		} else {
		    retval =
			kinematicsInverse(&pos, joints, &iflags, &fflags);
		    printf("%f %f %f %f %f %f\n", joints[0], joints[1],
			joints[2], joints[3], joints[4], joints[5]);
		    if (0 != retval) {
			printf("inv kins error %d\n", retval);
		    } else {
			retval =
			    kinematicsForward(joints, &pos, &fflags, &iflags);
			printf("%f %f %f %f %f %f\n", pos.tran.x, pos.tran.y,
			    pos.tran.z, pos.a, pos.b, pos.c);
			if (0 != retval) {
			    printf("fwd kins error %d\n", retval);
			}
		    }
		}
	    }
	} else {
	    if (jacobian) {
		if (12 != sscanf(buffer,
			"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
			&joints[0], &joints[1], &joints[2], &joints[3],
			&joints[4], &joints[5], &jointvels[0], &jointvels[1],
			&jointvels[2], &jointvels[3], &jointvels[4],
			&jointvels[5])) {
		    printf("?\n");
		} else {
//FIXME-AJ
//disabled for now        retval = jacobianForward(joints, jointvels, &pos, &vel);
		    printf("%f %f %f %f %f %f\n",
			vel.tran.x,
			vel.tran.y, vel.tran.z, vel.a, vel.b, vel.c);
		    if (0 != retval) {
			printf("fwd kins error %d\n", retval);
		    } else {
//FIXME-AJ
//disabled for now          retval = jacobianInverse(&pos, &vel, joints, jointvels);
			printf("%f %f %f %f %f %f\n",
			    jointvels[0],
			    jointvels[1],
			    jointvels[2],
			    jointvels[3], jointvels[4], jointvels[5]);
			if (0 != retval) {
			    printf("inv kins error %d\n", retval);
			}
		    }
		}
	    } else {
		if (6 != sscanf(buffer, "%lf %lf %lf %lf %lf %lf",
			&joints[0],
			&joints[1],
			&joints[2], &joints[3], &joints[4], &joints[5])) {
		    printf("?\n");
		} else {
		    retval =
			kinematicsForward(joints, &pos, &fflags, &iflags);
		    printf("%f %f %f %f %f %f\n", pos.tran.x, pos.tran.y,
			pos.tran.z, pos.a, pos.b, pos.c);
		    if (0 != retval) {
			printf("fwd kins error %d\n", retval);
		    } else {
			retval =
			    kinematicsInverse(&pos, joints, &iflags, &fflags);
			printf("%f %f %f %f %f %f\n", joints[0], joints[1],
			    joints[2], joints[3], joints[4], joints[5]);
			if (0 != retval) {
			    printf("inv kins error %d\n", retval);
			}
		    }
		}
	    }
	}
    }

    return 0;

#undef ITERATIONS
#undef BUFFERLEN
}

#endif /* ULAPI */

