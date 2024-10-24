#ifndef _FOC_POSITION_LOOP_H
#define _FOC_POSITION_LOOP_H


#define position_amp_val (2000)

/* motor position Acc/Dec step */
#define UPDNLIMT_SPEED(Ref,Target,step)	\
				(((Ref)>=(Target))?((((Ref)<=(Target+step))?(Target):(Ref-step))):(Ref+step)) 
			
extern void foc_Torque_position_loop(void);
			
extern void foc_position_loop(void);

extern int foc_check_position_fbk(void);

extern float pos_rad_to_encode(float rad);

extern float pos_encode_to_rad(float encode);

	
#endif