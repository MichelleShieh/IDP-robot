/***
 * TO DO LIST:
 * PI control
 * Move a certain distance
 * 
 * TO SOLVE:
 * line following without the line, turn right a bit, to make sure it wont ignore the first position to place tomatoes' trays
 * 
 * FINISHED?
 * left wheel moves when set to be stopped(solved by setting moving backwards)
 * the time condition for the line sensor to detect the end of the turning
***/


#include <iostream>
#include <fstream>
#include <robot_instr.h>
#include <robot_link.h>
#include <robot_delay.h>
#include <stopwatch.h>

#define ROBOT_NUM 9
#define high_power 85
#define low_power 0
robot_link rlink;			// datatype for the robot link
stopwatch watch;			// datatype for the stopwatch

const unsigned int PORT_0_READ_BITS = 0x0F;
/* Port 0 bits 0 to 3 are sensors to be read.
 * These bits have to be set to 1 to allow reading.
 * For each write, bitor the intended value with PORT_0_READ_BITS
 * to avoid writing 0 to these bits.
 */

using namespace std;
#ifndef START
	#define START
	ofstream fout("output.txt");
#else
	ofstream fout("output.txt",fstream::app);
#endif

struct reading{
	int pin[8];
};

struct speed {
	int l,r;
};

/*** start of communication part ***/
bool link_robot() {
	#ifdef __arm__
	  if (!rlink.initialise("127.0.0.1")){
	#else
	  if (!rlink.initialise(ROBOT_NUM)){
	#endif
		  cout<<"Cannot initialise link"<<endl;
		  return false;
	  }

  int val = rlink.request(TEST_INSTRUCTION);
  if (val == TEST_INSTRUCTION_RESULT) {
	  cout<<"connected"<<endl;
	  return true;
  }
  else if (val==REQUEST_ERROR) {
	cout<<"Fatal errors on link:"<<endl;
	rlink.print_errs();  
  }
  else cout<<"Test failed (bad value returned)"<<endl;	
  return false;
}
/*** end of communication part ***/

/*** helper function ***/
reading get_ic_reading(int result) {
	reading ic;
	result = result bitand 255;
    for (int i=0;i<8;i++) {
        ic.pin[i]=result%2;
        result/=2;
    }
	return ic;
}

void change_movement(int l,int r) {
	rlink.command(MOTOR_1_GO,l);
	rlink.command(MOTOR_2_GO,r);
}
/*** end of helper functinos ***/


/*** start of route part ***/
//use MOTOR_x, and MOTOR_y to move and turn
int check_status(bool sensor1, bool sensor2, bool sensor3, int dist) {
	/***
	 * Input: 
	 * 		the result from line following sensors 1->3: Assume 0 is white, 1 is black?
	 *		distance from the wall
	 * Output:
	 * 		return 0: out of line, 
	 * 		return 1: inline,
	 * 		return 2: junction, 
	 * 		return 3: need to turn right, 
	 * 		return 4: need to turn left.
	***/
}

speed pid_control() {
	//not sure if we need it or not
}

void deal_with_junction() {
	while(true) {
		int result = rlink.request(READ_PORT_0);
		reading ic = get_ic_reading(result);
		int line_reading = ic.pin[0]+ic.pin[1]*2+ic.pin[2]*4;
		fout<<line_reading<<":";
		if (line_reading!=7) {
			if(line_reading == 2) {
				change_movement(high_power+6,high_power+128); //go straight
				fout<<"go straight with line"<<endl;
			}
			else if(line_reading == 3) {
				change_movement(low_power+6,high_power+128); //110, turn left slightly
				fout<<"move towards left slightly"<<endl;
			}
			else if(line_reading == 1) {
				change_movement(low_power+6,high_power+128); //100, turn left 
				fout<<"move towards left"<<endl;
			}
			else if(line_reading == 6) {
				change_movement(high_power+6,low_power+128);//011, turn right slightly
				fout<<"move towards right slightly"<<endl;
			}
			else if(line_reading == 4) {
				change_movement(high_power+6,low_power+128);//001, turn right
				fout<<"move towards right"<<endl;
			}
			return;
		}
	}
}

void turn_right() {
	fout<<"turn right"<<endl;
	/*
	watch.start();
	while (watch.read()<4300) {
		change_movement(high_power,0);
	}
	*/
	while (true) {
		change_movement(high_power+20,20);
		delay(30);
		int result = rlink.request(READ_PORT_0);
		reading ic = get_ic_reading(result);
		int line_reading = ic.pin[0]+ic.pin[1]*2+ic.pin[2]*4;
		fout<<line_reading<<":";
		if(line_reading==2) {
			change_movement(high_power+6, high_power+128);
			break;
		}
	}
}

void turn_left() {
	fout<<"turn left"<<endl;
	//cout<<"turn left!!!!!!!"<<endl;
	/*
	watch.start();
	while (watch.read()<4300) {
		change_movement(0,high_power+128);
	}
	watch.stop();
	*/
	watch.start();
	while (true) {
		change_movement(20+128,high_power+20+128);
		//delay(30);
		int result = rlink.request(READ_PORT_0);
		reading ic = get_ic_reading(result);
		int line_reading = ic.pin[0]+ic.pin[1]*2+ic.pin[2]*4;
		fout<<line_reading<<": time passed "<<watch.read()<<endl;
		if(line_reading==2 && watch.read()>800) {
			change_movement(high_power+6, high_power+128);
			watch.stop();
			return;
		}
	}
	
}

void route(int cnt){
	//main part to control the route of the robot
	//TODO: currently just finish the task without going back
	while(true) {
		int result = rlink.request(READ_PORT_0);
		reading ic = get_ic_reading(result);
		int line_reading = ic.pin[0]+ic.pin[1]*2+ic.pin[2]*4;
		fout<<line_reading<<":";
		if (line_reading == 0) {
			fout<<"not inline, go straight without line"<<endl;
			change_movement(high_power+15,high_power+128); //go straight
		}
		if(line_reading == 2) {
			change_movement(high_power+6,high_power+128); //go straight
			fout<<"go straight with line"<<endl;
		}
		else if(line_reading == 3) {
			change_movement(low_power+6,high_power+128); //110, turn left slightly
			fout<<"move towards left slightly"<<endl;
		}
		else if(line_reading == 1) {
			change_movement(low_power+6,high_power+128); //100, turn left 
			fout<<"move towards left"<<endl;
		}
		else if(line_reading == 6) {
			change_movement(high_power+6,low_power+128);//011, turn right slightly
			fout<<"move towards right slightly"<<endl;
		}
		else if(line_reading == 4) {
			change_movement(high_power+6,low_power+128);//001, turn right
			fout<<"move towards right"<<endl;
		}
		else if(line_reading == 7) {
			/*** when met with junction
			 * Juntion start with number 0
			 * case: turn left at junction 3,5
			 * case: stop at vines, i.e.junction 1,2//TODO
			 * case: turn left at junction 6,7, and turn right back to the line
			 * 8, reverse
			 ***/
			fout<<"cnt of junction:"<<cnt<<endl;
			if (cnt == 3 || cnt == 5) {
				turn_left();
				cnt++;
			}
			else if(cnt==-1) {
				//testing for turn right
				turn_right();
			}
			else if(cnt==-2) {
				//testing for turn left
				turn_left();
			}
			/*
			else if (cnt==1 || cnt==2) {
				//at the vine
				change_movement(0,0);
				delay(3000);
				cnt++;
			}
			else if (cnt==6 || cnt==7) {
				turn_left();
				change_movement(0,0);
				delay(3000);
				turn_right();
				cnt++;	
			}
			* */
			else {
				deal_with_junction();
				cnt++;
			}
			/* one way to detect each junction once
			if (cnt!=3 && cnt!= 5) {
				delay(400);
			}*/
		}
		//delay(45);
	}
}
/*** end of route part ***/



/*** start of pick&classify part ***/
void pick(bool color) {
	/***
	 * Input: if having green color or not
	 * make command to actuator: pick 1, not pick 0
	 ***/
	// green: color == true; red: color == false
	// actuator: port 0 bit 6
	if(!color) {
		rlink.command(WRITE_PORT_0, (1 << 6) | PORT_0_READ_BITS);
	}
}
void push_tomato(int period) {
	//make command to actuator
	// actuator: port 0 bit 7
	rlink.command(WRITE_PORT_0, (1 << 7) | PORT_0_READ_BITS);
	delay(period);
	rlink.command(WRITE_PORT_0, 0 | PORT_0_READ_BITS);
}
/*** end of pick&classify part ***/


/*** start of push_tray part***/
enum led_status_t {
	/**
	 * This enum is arranged such that
	 * bit 0 of the enum corresponds to bit 4 of the write port (LED1)
	 * and bit 1 of enum corresponds to bit 5 of the write port (LED2).
	 * This allows us to do a simple bitshift to get the value to write.
	 */
	FULLSIZED, UNDERSIZED, CHERRY, IDLE
	// 00,      10,         01,     11
};
inline void update_indicator_led(enum led_status_t status) {
	rlink.command(WRITE_PORT_0, (status << 4) | PORT_0_READ_BITS);
}
void push_tray(int period, int motor_num) {
	/***
	 * Input: how long will it push the tray once, and the motor_num to do the push
	 * Control the motor the do the push work
	 ***/
}
/*** end of push part ***/

void test(){
	//do the test for all separate functions with artificial input
}


int main(){
	if (link_robot()) {
		/* Initialise read/write ports.
		 * Note it is unnecessary to initialise PORT_0_READ_BITS
		 * as this will be done within update_indicator_led(). */
		update_indicator_led(IDLE);
		// initialise write bits for actuators here if necessary
		/*** TESTING PARAMETER:
		 * -1 testing turn right
		 * -2 testing turn left
		 * -3 for going a certain distance
		 ***/
		route(-2);	
		int zzz;
		cin>>zzz;
	}
	fout.close();
	return 0;
}
