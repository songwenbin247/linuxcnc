/********************************************************************
* Description: genserkins.h
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

*/

/*
  genserkins.h
*/

#ifndef UARMKINS_3D_H
#define UARMKINS_3D_H

#include "gomath.h"		/* go_pose */
#include "hal.h"		/* HAL data types */

/*!
  The maximum number of joints supported by the general serial
  kinematics. Make this at least 6; a device can have fewer than these.
*/
#define UARM_LINKS 7
#define UARM_JOINTS 6
#define PI_2 GO_PI_2

#define DEFAULT_D0 112
#define DEFAULT_D1 15
#define DEFAULT_D2 142.5
#define DEFAULT_D3 159
#define DEFAULT_D4 72
#define DEFAULT_D5 60
#define DEFAULT_D6 45

typedef struct {

} uarm_struct;

extern int uarm_kin_size(void);

extern int uarm_kin_init(void);
/*
extern const char * uarm_kin_get_name(void);

extern int uarm_kin_num_joints(void * kins);

extern int uarm_kin_fwd(void * kins,
				const go_real *joint,
				go_pose * world);

extern int uarm_kin_inv(void * kins,
				const go_pose * world,
				go_real *joint);

extern int uarm_kin_set_parameters(void * kins, go_link * params, int num);

extern int uarm_kin_get_parameters(void * kins, go_link * params, int num);

extern int uarm_kin_jac_inv(void * kins,
				    const go_pose * pos,
				    const go_screw * vel,
				    const go_real * joints,
				    go_real * jointvels);


extern int uarm_kin_jac_fwd(void * kins,
				    const go_real * joints,
				    const go_real * jointvels,
				    const go_pose * pos,
				    go_screw * vel);

extern int uarm_kin_fwd_interations(genser_struct * genser);



  Extras, not callable using go_kin_ wrapper but if you know you have
  linked in these kinematics, go ahead and call these for your ad hoc
  purposes.


! Returns the number of iterations used during the last call to the
  inverse kinematics functions
extern int uarm_kin_inv_iterations(genser_struct * genser);

! Sets the maximum number of iterations to use in future calls to
  the inverse kinematics functions, after which an error will be
  reported
extern int uarm_kin_inv_set_max_iterations(genser_struct * genser, int i);

! Returns the maximum number of iterations that will be used to
 compute inverse kinematics functions
extern int uarm_kin_inv_get_max_iterations(genser_struct * genser);*/

#endif

