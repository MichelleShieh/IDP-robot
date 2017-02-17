/***
 * TO DO LIST:
 * Reverse Route
 * Picking
 * Placing the trays
 * PI control?
 * Move a certain distance
 * 
 * TO SOLVE:
 * 
 * TO TEST:
 * Move a certain distance
 * 
 * FINISHED?
 * left wheel moves when set to be stopped(solved by setting moving backwards)
 * the time condition for the line sensor to detect the end of the turning
 * line following without the line, turn right a bit, to make sure it wont ignore the first position to place tomatoes' trays
 * 
 * sub system demonstration:
 * 1.leave the starting box, follow the line on the vine sides
 * 2.turn left after passing all vine junctions, go straight without line(actually turn right a little bit to avoid ignoring the first junction for placing tomatoes), turn left after detecting the line, which is regarded the end of route out of line
 * 3.go a certain distance after the distance sensor detected the board where the tomatoes are haning, the distance are set as that between five possible position for tomatoes
***/


#include <iostream>
#include <fstream>
#include <robot_instr.h>
#include <robot_link.h>
#include <robot_delay.h>
#include <stopwatch.h>

#define ROBOT_NUM 9
#define high_power 100
#define low_power 70
#define diff 15
//turn left: increase the diff, turn right: reduce the diff; diff>25
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

/*** start of pick&classify part ***/
void pick(bool color) {
	/***
	 * Input: if having green color or not
	 * make command to actuator: pick 1, not pick 0
	 ***/
	// green: color == true; red: color == false
	// actuator: port 0 bit 6
	if(!color) {
		rlink.command(WRITE_PORT_0, (1 << 7) | PORT_0_READ_BITS);
	}
}

void reverse() {
	rlink.command(WRITE_PORT_0, (0 << 6) | PORT_0_READ_BITS);
	rlink.command(WRITE_PORT_0, (0 << 7) | PORT_0_READ_BITS);
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

/*** start of route part ***/
//use MOTOR_1, and MOTOR_2 to move and turn

void pi_control() {
	//PI control for line following
}

void deal_with_junction() {
	int line_reading;
	do {
		int result = rlink.request(READ_PORT_0);
		reading ic = get_ic_reading(result);
		line_reading = ic.pin[0]+ic.pin[1]*2+ic.pin[2]*4;
		fout<<line_reading<<":";
		switch(line_reading) {
		case 2:
			change_movement(high_power+diff,high_power+128); //go straight
			fout<<"go straight with line"<<endl;
			break;
		case 3:
			change_movement(low_power+diff,high_power+128); //110, turn left slightly
			fout<<"move towards left slightly"<<endl;
			break;
		case 1:
			change_movement(low_power+diff,high_power+128); //100, turn left 
			fout<<"move towards left"<<endl;
			break;
		case 6:
			change_movement(high_power+diff,low_power+128);//011, turn right slightly
			fout<<"move towards right slightly"<<endl;
			break;
		case 4:
			change_movement(high_power+diff,low_power+128);//001, turn right
			fout<<"move towards right"<<endl;
			break;
		}
	} while(line_reading == 7);
}

void turn_right() {
	fout<<"turn right"<<endl;
	/*
	watch.start();
	while (watch.read()<4300) {
		change_movement(high_power,0);
	}
	*/
	//change_movement(high_power+128,high_power);
	//delay(500);
	watch.start();
	while (true) {
		change_movement(high_power+17,40);
		//delay(30);
		int result = rlink.request(READ_PORT_0);
		reading ic = get_ic_reading(result);
		int line_reading = ic.pin[0]+ic.pin[1]*2+ic.pin[2]*4;
		fout<<line_reading<<":";
		if(line_reading==2 && watch.read()>800) {
			change_movement(high_power+diff, high_power+128);
			watch.stop();
			return;
		}
	}
}

void turn_right_without_junction() {
	watch.start();
	while (watch.read()<4200) {
		change_movement(high_power+17,40);
	}
	change_movement(high_power,high_power+128);
	watch.stop();
}

void turn_right_with_going_backwards() {
	change_movement(high_power+128,high_power);
	delay(500);
	watch.start();
	while (true) {
		change_movement(high_power+17,40+128);
		//delay(30);
		int result = rlink.request(READ_PORT_0);
		reading ic = get_ic_reading(result);
		int line_reading = ic.pin[0]+ic.pin[1]*2+ic.pin[2]*4;
		cout<<line_reading<<endl;
		fout<<line_reading<<":";
		if (watch.read()>800) {
			switch(line_reading) {
			case 2:
				change_movement(high_power+diff,high_power+128); //go straight
				fout<<"go straight with line"<<endl;
				break;
			case 3:
				change_movement(low_power,high_power+128); //110, turn left slightly
				fout<<"move towards left slightly"<<endl;
				break;
			case 1:
				change_movement(low_power+128,high_power+128); //100, turn left 
				fout<<"move towards left"<<endl;
				break;
			case 6:
				change_movement(high_power,low_power+128);//011, turn right slightly
				fout<<"move towards right slightly"<<endl;
				break;
			case 4:
				change_movement(high_power,low_power);//001, turn right
				fout<<"move towards right"<<endl;
				break;
			}
			watch.stop();
			return;
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
	//change_movement(high_power+128,high_power);
	//delay(500);
	watch.start();
	while (true) {
		change_movement(40+128,high_power+17+128);
		//delay(30);
		int result = rlink.request(READ_PORT_0);
		reading ic = get_ic_reading(result);
		int line_reading = ic.pin[0]+ic.pin[1]*2+ic.pin[2]*4;
		fout<<line_reading<<": time passed "<<watch.read()<<endl;
		if(line_reading==2 && watch.read()>800) {
			change_movement(high_power+diff, high_power+128);
			watch.stop();
			return;
		}
	}
	
}


void error_handling(int prev_detection) {
	/***
	 * error handling part:
	 * when it goes to dark place, check the last detection
	 * if it was turning right, keeps turning left, until go back to non-all-dark places
	 * if it was turning left, keeps turning right, until go back to non-all-dark places
	 ***/
	 fout<<"INTO ERROR HANDLING"<<endl;
	 if (prev_detection == 3 || prev_detection ==1 || prev_detection==7 || prev_detection==2) {
		 while (1) {		
			 int result = rlink.request(READ_PORT_0);
			 reading ic = get_ic_reading(result);
			 int detection = ic.pin[0]+ic.pin[1]*2+ic.pin[2]*4;
			 if (detection!=0) {
				 break;
			 }
		 }
	 }
	 else if (prev_detection == 4 || prev_detection== 5) {
		 while (1) {		
			 int result = rlink.request(READ_PORT_0);
			 reading ic = get_ic_reading(result);
			 int detection = ic.pin[0]+ic.pin[1]*2+ic.pin[2]*4;
			 if (detection!=0) {
				 break;
			 }
		 }
	 }
}


void route(int cnt){
	//main part to control the route of the robot
	//TODO: currently just finish the task without going back
	int dist;
	//int flag[10]= {false};
	bool is_picked[10] = { false };
	int prev_detection=0;
	int picking_time[5] = {1000,-1,1000,1000,1000};
	//bool flag=false; //after detecting the board for picking, become true;
	//bool reverse=false; //after finish the main route, turn to false, and do the reverse
	while(true) {
		dist=rlink.request(ADC0);
		fout<<"dist:"<<dist<<endl;	
		int result = rlink.request(READ_PORT_0);
		reading ic = get_ic_reading(result);
		int line_reading = ic.pin[0]+ic.pin[1]*2+ic.pin[2]*4;
		fout<<line_reading<<":";
		if ((cnt==0 || cnt==1) && dist>80) {
			fout<<"See board"<<endl;
			change_movement(high_power+diff,high_power);
			watch.start();
			for (int i=0;i<5;i++) {
				if (i==1) continue;
				fout<<"time:"<<watch.read()<<endl;
				if (!is_picked[i] && watch.read()>picking_time[i]) {
					change_movement(0,0);
					is_picked[0]=true;
					delay(2000); 
					pick(0); //TODO: add the part for colour detection
					watch.stop();
					watch.start();
					while (1) {
						change_movement(high_power+128,high_power);
						if (watch.read()>picking_time[i]+50) {
							change_movement(0,0);
							delay(2000);
							reverse();
							watch.stop();
							break;
						}
					}
				}
			}
			if (is_picked[0] && is_picked[2] && is_picked[3] && is_picked[4]) {
				cnt++;
				for (int i=0;i<5;i++) {
					is_picked[i]=false;
				}
			}
		}
		else {
			switch(line_reading) {
			case 0:
				fout<<"not inline, go straight without line"<<endl;
				//change_movement(high_power+diff+10,high_power+128); //go straight
				if (cnt!=4 && cnt!=10) {
					error_handling(prev_detection);
				}
				if (cnt<8) {
					//use the distance sensor to help keep straight
					if(dist < 50) { // turn right
						change_movement(high_power,low_power);//turn right
						fout<<"move towards right"<<endl;
					}
					else if(dist > 60) {
						change_movement(low_power,high_power+128); //110, turn left slightly				
						fout<<"move towards left slightly"<<endl;
					} 
					else { // go straight
						change_movement(high_power+diff+10,high_power+128); //go straight
					}
					delay(30);
				}
				else {
					change_movement(high_power+diff+10,high_power+128);
				}

				break;
			case 2:
				change_movement(high_power+diff,high_power+128); //go straight
				fout<<"go straight with line"<<endl;
				break;
			case 3:
				change_movement(low_power,high_power+128); //110, turn left slightly
				fout<<"move towards left slightly"<<endl;
				break;
			case 1:
				change_movement(low_power+128,high_power+128); //100, turn left 
				fout<<"move towards left"<<endl;
				break;
			case 6:
				change_movement(high_power,low_power+128);//011, turn right slightly
				fout<<"move towards right slightly"<<endl;
				break;
			case 4:
				change_movement(high_power,low_power);//001, turn right
				fout<<"move towards right"<<endl;
				break;
			case 7:
				/*** when met with junction
				 * Juntion start with number 0
				 * case: turn left at junction 3,5
				 * case: stop at vines, i.e.junction 1,2//TODO
				 * case: turn left at junction 6,7, and turn right back to the line 
				 * case: junction 8, reverse
				 ***/
				fout<<"cnt of junction:"<<cnt<<endl;
				switch(cnt) {
				case 3:
				case 5:
				case -2: // testing for turn left
				case 12:
					turn_left();
					break;
				case -1: // testing for turn right
				case 11:
					turn_right();
					break;
				/*
				case 1:
				case 2:
					//at the vine
					change_movement(0,0);
					delay(3000);
					break;
				case 6:
				case 7:
					//placing
			// TODO
					turn_left();
					change_movement(high_power+128,high_power);
					delay(1000);
					//TODO: drop
					turn_right();
					break;
				*/
				case 8:
					//start of reverse
					turn_left();
					change_movement(0,0);
					delay(1000);
					change_movement(high_power+128,high_power);
					delay(2200);
					turn_left();
					break;
				case 10:
					/*
					change_movement(high_power,high_power+128);
					delay(2000);
					*/
					turn_right_without_junction();
					break;
				case 13:
					turn_right_with_going_backwards();
					break;
				case 15:
					change_movement(0,0);
					return;
				case 0:
				case 1:
					deal_with_junction();
				/* one way to detect each junction once
				if (cnt!=3 && cnt!= 5) {
					delay(400);
				}*/
				}
				if(cnt >= 0) { // not a test case
					cnt++;
				}
				//delay(45);
			}
			prev_detection = line_reading;
		}
	}
}

/*** end of route part ***/


void test(){
	//do the test for all separate functions with artificial input
	watch.start();
	for(int i=0;i<100;i++) {
		rlink.request(ADC0);
	}
	//361 for running on computer
	//158 for running on microprocessor
	int time=watch.read();
	cout<<time<<endl;
}

void test_picking() {
}

void test_placing() {
}

void test_actuator() {
	while(1){
		rlink.command(WRITE_PORT_0, 255);
		delay(3000);
		rlink.command(WRITE_PORT_0, PORT_0_READ_BITS);
		delay(3000);
	}
}

void test_conveyor() {
	//Blue wire
	rlink.command(MOTOR_3_GO,100);
}

void test_going_straight_without_line() {
	while(1) {
		int dist=rlink.request(ADC0);
		fout<<"dist:"<<dist<<endl;	
		int result = rlink.request(READ_PORT_0);
		reading ic = get_ic_reading(result);
		int line_reading = ic.pin[0]+ic.pin[1]*2+ic.pin[2]*4;
		fout<<line_reading<<":";	
		switch(line_reading) {
		case 0:
			fout<<"not inline, go straight without line"<<endl;
			//change_movement(high_power+diff+10,high_power+128); //go straight
			/*
			if(dist < 50) { // turn right
				//change_movement(128,0+128);//011, turn right
				change_movement(high_power+diff,low_power);//turn right
				fout<<"move towards right"<<endl;
			}
			else if(dist > 60) {
				//change_movement(low_power+diff,128+128); //110, turn left slightly
				change_movement(low_power+diff,high_power+128); //110, turn left slightly				
				fout<<"move towards left slightly"<<endl;
			} 
			else {*/ 
			// go straight
			change_movement(high_power+diff,high_power+128); //go straight
			delay(30);
			break;
		case 2:
			change_movement(high_power+diff,high_power+128); //go straight
			fout<<"go straight with line"<<endl;
			break;
		case 3:
			change_movement(low_power+diff,high_power+128); //110, turn left slightly
			fout<<"move towards left slightly"<<endl;
			break;
		case 1:
			change_movement(low_power+128,high_power+128); //100, turn left 
			fout<<"move towards left"<<endl;
			break;
		case 6:
			change_movement(high_power+diff,low_power+128);//011, turn right slightly
			fout<<"move towards right slightly"<<endl;
			break;
		case 4:
			change_movement(high_power+diff,low_power);//001, turn right
			fout<<"move towards right"<<endl;
			break;
		}
	}
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
		//route(0);
		//change_movement(high_power+diff+10,high_power+128);
		test_going_straight_without_line();
		//route(0);
		//turn_right_without_junction();
		//turn_right_with_going_backwards();
		//test();	
		//test_actuator();
		//test_conveyor();
		int zzz;
		cin>>zzz;
	}
	fout.close();
	return 0;
}
