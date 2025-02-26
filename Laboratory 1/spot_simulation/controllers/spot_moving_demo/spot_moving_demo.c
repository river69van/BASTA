#include <webots/robot.h>
#include <webots/motor.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define NUMBER_OF_JOINTS 12
#define TIME_STEP 32



//abduction motor	= -0.6 < range < 0.5 
//rotation motor	= -1.7 < range < 1.7
//elbow motor		= -0.45 < range < 1.6  



//var adjustments for proper controlls
double pi = 3.14159265359;
double STEP_FREQ = 1;
double elbow_max_amplitude_value = 1;
double elbow_min_amplitude_value = -0.45;
double shoulder_max_amplitude_value = 1;
double shoulder_min_amplitude_value = -1.7; 
double abduction = 0.3;							//guti na pag bira han mga legs para dre ma tumba hahaha


//bool soft_start_flag = true;

static WbDeviceTag motors[NUMBER_OF_JOINTS];


//leg names 
static const char *motor_names[NUMBER_OF_JOINTS] = {
  "front left shoulder abduction motor",  "front left shoulder rotation motor",  "front left elbow motor",
  "front right shoulder abduction motor", "front right shoulder rotation motor", "front right elbow motor",
  "rear left shoulder abduction motor",   "rear left shoulder rotation motor",   "rear left elbow motor",
  "rear right shoulder abduction motor",  "rear right shoulder rotation motor",  "rear right elbow motor"};



//to display the move position 
static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(0);
  }
}



//structure requirements kada leg
typedef struct {
	//double leg_num;
	
	
	//select the motor position 
	int shoulder_abduction_motor;
	int shoulder_rotation_motor;
	int elbow_motor;
	
	
	//select the abs max amplitude kada motor
	double shoulder_abduction_max;
	double shoulder_rotation_max;
	double elbow_max;
	
	double shoulder_abduction_min;		//unused
	double shoulder_rotation_min;		//unused
	double elbow_min;					//unused
	
	
	
	//delay kada motor karan pag lakat
	double phase_shift_shoulder_abduction;
	double phase_shift_shoulder_rotation;
	double phase_shift_elbow;
	//double frequency;

}LEG;



/*
//test code
static void movement_decomposition(const double *target, double duration) {
  const double time_step = wb_robot_get_basic_time_step();
  const int n_steps_to_achieve_target = duration * 1000 / time_step;
  double step_difference[NUMBER_OF_JOINTS];
  double current_position[NUMBER_OF_JOINTS];

  for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
    current_position[i] = wb_motor_get_target_position(motors[i]);
    step_difference[i] = (target[i] - current_position[i]) / n_steps_to_achieve_target;
  }

  for (int i = 0; i < n_steps_to_achieve_target; ++i) {
    for (int j = 0; j < NUMBER_OF_JOINTS; ++j) {
      current_position[j] += step_difference[j];
      wb_motor_set_position(motors[j], current_position[j]);
    }
    step();
  }
}
*/

//set values to zero for standing pos
void initialize_motors() {	
	for (int i = 0; i < NUMBER_OF_JOINTS; ++i)	
		motors[i] = wb_robot_get_device(motor_names[i]);


	for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
		wb_motor_set_position(motors[i], 0);
		step();
	}
}


/*


//test codes
static void SPLIT(double duration) {
  const double motors_target_pos[NUMBER_OF_JOINTS] = {-0.09, 0, 0,   // Front left leg
                                                      0.09,  0, 0,   // Front right leg
                                                      0, -0.5, 0,   // Rear left leg
                                                      0,  -0.5, 0};  // Rear right leg
  movement_decomposition(motors_target_pos, duration);
}





void elbows(){
	
	
	
}


void take_step(double phase_shift, double initial_time_ref) {
	
	double RandL1_shift = pi/8; 
	double RandL2_shift = 0; 
	double shift = pi/16;
	
	
	
	double increment = wb_robot_get_time() - initial_time_ref;
  //double time = wb_robot_get_time();
	const double elbow_sine_func2 = sin(2 * increment * (STEP_FREQ) + phase_shift +  0.6 - (pi/2) + shift);
	const double elbow_sine_func1 = sin(2 * increment * (STEP_FREQ ) + phase_shift + 0.6 + (pi/2) + shift);
	double total_pos1 = elbow_max_amplitude_value * elbow_sine_func1;
	double total_pos2 = elbow_max_amplitude_value * elbow_sine_func2;
	printf("elbow_sine_func1 = %fl\n", elbow_sine_func1);
	if(total_pos1 < -0.45)
		total_pos1 = -0.45;
	
	if(total_pos2 < -0.45)
		total_pos2 = -0.45;
	
	
	if(elbow_sine_func1 > 0){
		elbow_max_amplitude_value = 0.5;
		total_pos1 = elbow_max_amplitude_value * elbow_sine_func1;
		
		wb_motor_set_position(motors[elbow_right_back], total_pos1); 		// Rear right
		wb_motor_set_position(motors[elbow_left_front],  total_pos1); 		// Front left
		if(total_pos1 > 1.6)
		total_pos1 = 1.6;
	}
	if(elbow_sine_func1 < 0){
		elbow_max_amplitude_value = 0.45;
		total_pos1 = elbow_max_amplitude_value * elbow_sine_func1;
		
		wb_motor_set_position(motors[elbow_right_back], (total_pos1+ 0.05)); 		// Rear right
		wb_motor_set_position(motors[elbow_left_front],  total_pos1); 		// Front left
		if(total_pos1 < -0.45)
		total_pos1 = -0.45;
	}
	
	
	
	if(elbow_sine_func2 > 0){
		elbow_max_amplitude_value = 0.5;
		total_pos2 = elbow_max_amplitude_value * elbow_sine_func2;
		
		wb_motor_set_position(motors[elbow_right_front],  total_pos2); 		// Front right
		wb_motor_set_position(motors[elbow_left_back],  (total_pos2 + 0.05)); 		// Rear left
		if(total_pos2 > 1.6)
		total_pos2 = 1.6;
	}
	if(elbow_sine_func2 < 0){
		elbow_max_amplitude_value = 0.45;
		total_pos2 = elbow_max_amplitude_value * elbow_sine_func2;
		
		wb_motor_set_position(motors[elbow_right_front],  total_pos2); 		// Front right
		wb_motor_set_position(motors[elbow_left_back],  total_pos2); 		// Rear left
		if(total_pos2 < -0.45)
		total_pos2 = -0.45;
	}
	
	
	printf("POS1 value = %fl\n", total_pos1);
	printf("POS2 value = %fl\n", total_pos2);
	
	
	wb_motor_set_position(motors[shoulder_left_front],  shoulder_max_amplitude_value * sin(2 * wb_robot_get_time() * STEP_FREQ + phase_shift )); 		// Front left
	wb_motor_set_position(motors[shoulder_right_front],  shoulder_max_amplitude_value * sin(2 * increment * STEP_FREQ + phase_shift + 2*pi)); 			// Front right
	wb_motor_set_position(motors[shoulder_left_back],  shoulder_max_amplitude_value * sin(2 * increment * STEP_FREQ + phase_shift+ 5*pi/2)); 			// Rear left
	wb_motor_set_position(motors[shoulder_right_back], shoulder_max_amplitude_value * sin(2 * increment * STEP_FREQ + phase_shift + pi )); 		// Rear right
	
}


void front_left(LEG *frontLeft, double initial_time_ref){
	
	
}
void back_right(){}

*/


void leg_move(LEG *leg, double initial_time_ref, double frequency){
	
	
	double increment = wb_robot_get_time() - initial_time_ref;
	double range_checker = sin(2 *  pi  * increment * (frequency) + (leg->phase_shift_elbow));
	
	
		if(range_checker > 0)
			leg->elbow_max = 1.6; 
		if(range_checker < 0)
			leg->elbow_max = 0.45;
		
		wb_motor_set_position(motors[leg->shoulder_abduction_motor],  (leg->shoulder_abduction_max) * sin(2 * pi * increment * (frequency) + (leg->phase_shift_shoulder_abduction)));
		
		wb_motor_set_position(motors[leg->shoulder_rotation_motor],  (leg->shoulder_rotation_max) * sin(2 * pi * increment * (frequency) + (leg->phase_shift_shoulder_rotation)));
		
		wb_motor_set_position(motors[leg->elbow_motor],  (leg->elbow_max) * sin(2 * pi * increment * (frequency) + (leg->phase_shift_elbow)));
		
		//printf("time = %fl\n", 2 * pi * increment ); 	//pan debug 
		//printf("time = %fl\n", range_checker); 		//pan debug 
		//printf("time = %fl\n", (leg->shoulder_rotation_max) * sin(2 * pi * increment * (frequency) + (leg->phase_shift_shoulder_rotation))); 		//pan debug 
		
	
	
}

/*

void STEP1(LEG *leg1, LEG *leg2, LEG *leg3, LEG *leg4,double initial_time_ref, double frequency){
	
	
	
}
void STEP2(LEG *leg1, LEG *leg2, LEG *leg3, LEG *leg4,double initial_time_ref, double frequency){
	
	
	
}

*/


//baga han chetah na ka dalagan pero kingkoy hahahah
void walk(LEG *FRight, LEG *FLeft, LEG *BkRight, LEG *BLeft) {
    const double initial_time = wb_robot_get_time();


	// front left pati right 
	while (wb_robot_get_time() - initial_time < 0.5) {
		//take_step(0,initial_time );
		leg_move(FRight, initial_time, STEP_FREQ);
		leg_move(FLeft, initial_time, STEP_FREQ);
		//leg_move(BkRight, initial_time, STEP_FREQ);
		//leg_move(BLeft, initial_time, STEP_FREQ);
		step();
	}
	
	// back left pati right 
	while (wb_robot_get_time() - initial_time < 1) {
		//take_step(0,initial_time );
		//leg_move(FRight, initial_time, STEP_FREQ);
		//leg_move(FLeft, initial_time, STEP_FREQ);
		leg_move(BkRight, initial_time, STEP_FREQ);
		leg_move(BLeft, initial_time, STEP_FREQ);
		step();
	}
	
	
	
}



//Pag assign values ha struct
void init_legs(LEG *leg_location, 

	double shoulder_abduction_motor, 
	double shoulder_rotation_motor, 
	double elbow_motor,

	double shoulder_abduction_max, 
	double shoulder_rotation_max, 
	double elbow_max,

	double shoulder_abduction_min, 
	double shoulder_rotation_min, 
	double elbow_min, 

	double phase_shift_shoulder_abduction, 
	double phase_shift_shoulder_rotation, 
	double phase_shift_elbow){

	leg_location->shoulder_abduction_motor = shoulder_abduction_motor;
	leg_location->shoulder_rotation_motor = shoulder_rotation_motor;
	leg_location->elbow_motor = elbow_motor;

	leg_location->shoulder_abduction_max = shoulder_abduction_max;
	leg_location->shoulder_rotation_max = shoulder_rotation_max;
	leg_location->elbow_max = elbow_max;

	leg_location->shoulder_abduction_min = shoulder_abduction_min;
	leg_location->shoulder_rotation_min = shoulder_rotation_min;
	leg_location->elbow_min = elbow_min;

	leg_location->phase_shift_shoulder_abduction = phase_shift_shoulder_abduction;
	leg_location->phase_shift_shoulder_rotation = phase_shift_shoulder_rotation;
	leg_location->phase_shift_elbow = phase_shift_elbow;

}

int main() {
	
	
	wb_robot_init();
	initialize_motors();	//set values to zero 
	

	LEG right_front, left_front, right_back, left_back; //mga object instance kada leg
	
	
	
	//pag assign na mga values
	init_legs(&right_front,
	3,
	4,
	5,
	abduction,
	shoulder_max_amplitude_value,
	elbow_max_amplitude_value,
	abduction,
	shoulder_min_amplitude_value,
	elbow_min_amplitude_value,
	0,
	0,
	pi/2);
	
	
	init_legs(&left_front,
	0,
	1,
	2,
	-abduction,
	shoulder_max_amplitude_value,
	elbow_max_amplitude_value,
	abduction,
	shoulder_min_amplitude_value,
	elbow_min_amplitude_value,
	0,
	0,
	pi/2);
	
	
	init_legs(&left_back,
	6,
	7,
	8,
	abduction,
	shoulder_max_amplitude_value,
	elbow_max_amplitude_value,
	abduction,
	shoulder_min_amplitude_value,
	elbow_min_amplitude_value,
	0,
	0,
	pi/2);
	
	
	init_legs(&right_back,
	9,
	10,
	11,
	-abduction,
	shoulder_max_amplitude_value,
	elbow_max_amplitude_value,
	abduction,
	shoulder_min_amplitude_value,
	elbow_min_amplitude_value,
	0,
	0,
	pi/2);

	//pag loop na
	while(1){

		walk(&right_front, &left_front, &right_back, &left_back);
	}

	wb_robot_cleanup();
	return EXIT_SUCCESS;
}
