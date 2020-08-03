// MouseoVeR - software for interactive Virtual Reality
// Jeremy Cohen (2010-2014)

// this software can communicate (via serial port, RS-232) with the virtual
// reality game engine "MouseoVeR"

// this software can also communicate (via serial port, usb) with the matlab
// GUI "MouseoVeR_(ver).m"

// pin 13 on the Arduino Mega and chipKit max32 board is connected to a red LED
#define LED_PIN   13
/*
// BCS ver B hookup
// Solenoid hookup:
#define SOLENOID1 30
#define SOLENOID2 31
#define SOLENOID3 32
#define SOLENOID4 33
#define SOLENOID5 34
#define SOLENOID6 35
#define SOLENOID7 36
#define SOLENOID8 37

// Beam break circuit hookup:
#define BEAM1     38
#define BEAM2     39
#define BEAM3     40
#define BEAM4     41
#define BEAM5     42
#define BEAM6     43
#define BEAM7     44
#define BEAM8     45
*/

// BCS ver C hookup
#define SOLENOID1 29
#define SOLENOID2 28
#define SOLENOID3 27
#define SOLENOID4 26
#define SOLENOID5 25
#define SOLENOID6 24
#define SOLENOID7 23
#define SOLENOID8 22

#define BEAM1     37
#define BEAM2     36
#define BEAM3     35
#define BEAM4     34
#define BEAM5     33
#define BEAM6     32
#define BEAM7     31
#define BEAM8     30

// Digital I/O hookup:
// Reverse order compared to the PCB marking to make breakout bnc's more convienient for me: Jeremy
//#define DIGITAL1  69
//#define DIGITAL2  68
//#define DIGITAL3  67
//#define DIGITAL4  66
//#define DIGITAL5  65
//#define DIGITAL6  64
//#define DIGITAL7  63
//#define DIGITAL8  62

#define DIGITAL1  4 //69
#define DIGITAL2  5 //68
#define DIGITAL3  6 //67
#define DIGITAL4  8 //66
#define DIGITAL5  20 //65
#define DIGITAL6  21 //64
#define DIGITAL7  A0 //63
#define DIGITAL8  A1 //62

// Chip select pins used with the SPI interface
#define ADC_PIN   49
#define DAC1_PIN  53
#define DAC2_PIN  48

// this is how long the AP detection th-1 acctepance window is (5 = a window of 4-5 ms)
#define DELAY_TH1 2 // descending phase trigger must occur within 2ms from ascending phase 
// this is how long the AP detection th-2 blocking window is (5 = a window of 4-5 ms)
#define DELAY_TH2 2 // if high threshold detector is also triggered within 2ms from ascending phase trigger, ignore detection

#define LINE_BUFFER 128
#define VR_BUFFER 128
#define EVENT_STRING 32

#define ENDPOS 533000

// Zaber servomotor command numbers
#define HOME_CMD 1 // for "homepos" string
#define STORE_POS_CMD 16 // not used
#define MOVE_STORED_POS__CMD 18 // not used
#define MOVE_ABSOLUTE_CMD 20 // for "moveto" string
#define MOVE_RELATIVE_CMD 21 // not used
#define STOP_CMD 23 // for "servoStop" string
#define SET_SPEED_CMD 42 // for "setSpeed" string
#define SET_ACC_CMD 43 // for "setAccel" string
#define SET_MAX_POS_CMD 44 // for "setMaxPos" string
#define SET_MICROSTEP_RES_CMD 37 // for "setMicrostepRes" string


// include the spi library code:
#include <SPI.h>
// - and required BCSIII header files - 
#include <CS.h>            // !!! Mandatory header for SPI chip select routines
#include <MCP23S17.h>      // !!! Mandatory header for Beam break I/O
#include <BCSIII.h>        // !!! Mandatory header file for BCSII main board functions 
#include <DRIVER.h>        // header file for solendoid driver panel functions
#include <ADC_AD7328.h>    // header file for ADC panel functions
#include <DAC_AD57x4.h>    // header file for DAC panel functions
#include <LCD2x16.h>       // header file for LCD panel functions


ADC_AD7328 adc;    // instantiate ADC panel 
DAC_AD57x4 dac;    // instantiate DAC panel 
LCD2x16 lcd;       // instantiate LCD panel  
DRIVER driver;     // instantiate driver panel



////////////
// VARIABLES
////////////

int32_t new_ms, old_ms = 0; // ms clocks
int32_t loopTime_us, loopTimeMax = 0; // us clocks

uint16_t pcrxovf = 0, vrrxovf = 0; // overflow buffers for pc and vr computers

char buffer[LINE_BUFFER];  // character line input buffer
uint8_t idx; // counter for PC input buffer
char vrbuffer[VR_BUFFER];  // vr input buffer
char lastVrbuffer[VR_BUFFER]; // last saved vrbuffer
uint8_t vr_idx; // counter for VR input buffer

char stringBuff[128]; // buffer used for string printing

char *argv[32]; // input line arguments (as character strings)
int arg1, arg2, arg3; // input arguments (as integers)
int ain; // analog input value
uint8_t state;
uint16_t count;

// servomotor properties
int device;
long position;
long servoSpeed;
long maxPos;
long microstepRes;

// VR data variables
long vr_time; // vr time in ms
long vr_x, vr_y, vr_z; //vr coordinates in mm
long vr_speed; // vr speed in mm/s
long vr_direction; // head direction in hundreds of a degree, range = {-17999, 18000}
long vr_eventcnt = 0; // number of events (beam breaks, reward zones contacted etc. in VR within the previous frame (time window)
long speed_th = 1;
long speed_th_curr = 0;// speed threshold value for reward delivery schedules (RDS)
uint8_t speed_th_tag = 1; // tag for whether to auto set the speed_th (used during RDS_x4...). can turn off to manually set speed_th

// VR data for speed buffer (smoothed speed)
long speed_buffer[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 20 value speed buffer (=400ms)
long total_speed=0;
long avg_speed=0;
uint8_t speed_buffer_index = 0;

// variables for timer objects for AP spike detector inputs
uint8_t th1Timer; // timer for AP detection 
uint8_t th2Timer; // timer for high threshold detector (noise)
uint8_t th1Detected;
uint8_t th2Detected;
uint8_t thDetect;

uint8_t training;
long endpos, homepos;

// potential strings from VR data
char eventstr_1[EVENT_STRING], eventstr_2[EVENT_STRING], eventstr_3[EVENT_STRING], eventstr_4[EVENT_STRING];
char eventstr_curr[EVENT_STRING]; // tmp string used as current event

/////////////
// variables for behavioral contingency programs - reward delivery schedules.
/////////////

uint8_t RD_tag = 0; // tag for delivery of reward (RD). Turned to 1 after each RD delivery. Reset to 0 when the animal move to a new location. This is used to prevent multiple reward in lick-dependent RD protocol.

// RD_score - running average of the success of the animal in percentage of rewards/lap
uint8_t lap_tag=0;
uint32_t lap_cnt=0;
uint32_t lap_loop=2;
uint32_t RD_cnt_curr=0;
uint32_t RD_score=0;

uint8_t ii; //for looping in the random reward protocol
uint8_t jj; //for looping in the random reward protocol
long temp; //for swaping during sorting

long randNum; 
uint8_t rollMax = 1; // max number of rolls per new location, increase to increase chance of delivering RD
uint8_t rollMax_curr = 1; // counter
uint8_t randRD_flag = 0; // flag for whether to use random rewards. 
uint8_t randRD_roll_tag = 0; // tag to roll or not, set by entering new locations
uint32_t rollNum = 1; // counter
uint8_t min_RD=0; // minimum number of potential Rewards delivered per lap as defined by RDS (see below)
uint8_t min_RD_curr = 0; // current value to print when changed


uint8_t lk_tag = 0; // lick tag for lick dependent Reward delivery
uint8_t lick_RD = 0; // global tag for lick-dependent RD schedule. 1=ON,0=OFF
uint8_t lk_cnt = 0; // lick count for lick-dependent RD
uint8_t lk_th = 3; // lick threshold (number of licks the animal needs to accumulate to trigger a reward)
uint8_t RD_zone_tag = 0; //tag indicating that the animal is currently at a reward zone. This is only used for lick-dependent RD protocol. See eventstr_curr update.

// RDS stands for: Reward Delivery Schedule. _A stands for version, default state=0
// e.g. - RDS_A1 is a tag (if == 1, do x,y,z...)
uint32_t RD_cnt=0; // reward counter
uint8_t RDS_tag=0; // tag to disp RDS state on LCD monitor. if 1. disp ex, "RDS_A1 ON" (NOT CURRENTLY USED)

// counters for RDS control loops. 
uint8_t RDS_A1_cnt=0; uint8_t RDS_A2_cnt=0; uint8_t RDS_A3_cnt=0; uint8_t RDS_A4_cnt=0; uint8_t RDS_A14_cnt=0; // linear-track AND tunnel-track
uint8_t RDS_B1_cnt=0; uint8_t RDS_B2_cnt=0; uint8_t RDS_B3_cnt=0; uint8_t RDS_B4_cnt=0; uint8_t RDS_B14_cnt=0; // L-track
uint8_t RDS_C1_cnt=0; uint8_t RDS_C2_cnt=0; uint8_t RDS_C3_cnt=0; uint8_t RDS_C4_cnt=0; uint8_t RDS_C14_cnt=0; // fig8-track
uint8_t RDS_D1_cnt=0; uint8_t RDS_D2_cnt=0; uint8_t RDS_D3_cnt=0; uint8_t RDS_D4_cnt=0; uint8_t RDS_D14_cnt=0; // oval-track
uint8_t RDS_E1_cnt=0; uint8_t RDS_E2_cnt=0; uint8_t RDS_E3_cnt=0; uint8_t RDS_E4_cnt=0; uint8_t RDS_E14_cnt=0; // teardrop-track

///////////////////////////////////////////////////
//for Xinyu
uint8_t RDS_XZ1_cnt=0; uint8_t RDS_XZR_cnt=0; uint8_t RDS_XZ11_cnt=0; uint8_t RDS_XZ11_TR_cnt=0; uint8_t RDS_XZ2_cnt=0; uint8_t RDS_XZTR1_cnt=0; uint8_t RDS_XZTR2_cnt=0; uint8_t RDS_XZTR3_cnt=0;
uint8_t RDS_XZ3_cnt=0;  //for triangle maze
uint8_t RDS_XZ12_cnt=0;  //unique maze with only one reward location
uint8_t RDS_XZRand2_cnt=0; uint8_t RDS_XZRand3_cnt=0;  //random rewards with a fixed number of rewards

uint8_t new_event_tag=0;  //is 1 when the animal enters a new location
///////////////////////////////////////////////////

uint8_t RDS_A1 = 0; // RDS_A used in the linear-track AND tunnel-track
uint8_t RDS_A2 = 0;
uint8_t RDS_A3 = 0;
uint8_t RDS_A4 = 0;
uint8_t RDS_A14 = 0;
uint8_t RDS_A1_listnum = 0; // current position in RDS_A1_list
uint8_t RDS_A2_listnum = 0; // current position in RDS_A2_list
uint8_t RDS_A3_listnum = 0; // current position in RDS_A3_list
uint8_t RDS_A4_listnum = 0; // current position in RDS_A4_list
uint8_t RDS_A14_listnum = 0; // current position in RDS_A14_list


uint8_t RDS_x0_num = 34;  //I changed this to 33. In Jeremy's code, it's 37 (He has 37 photobeams in total). 
char *RDS_x0_list[] = {
  "p0","p1","p2","p3","p4","p5","p6","p7","p8","p9","p10",
  "p11","p12","p13","p14","p15","p16","p17","p18","p19",
  "p20","p21","p22","p23","p24","p25","p26","p27","p28","p29",
  "p30","p31","p32","p33"}
;

uint8_t RDS_A1_num = 12;
char *RDS_A1_list[] = {
  "p30", "p18", "p12", "p6",
  "p5", "p4", "p9", "p19", 
  "p25", "p31", "p32", "p0" }
;


uint8_t RDS_A2_num = 8;
char *RDS_A2_list[] = {
  "p18", "p12", "p6",
  "p5", "p19", 
  "p25", "p31", "p0" }
;
 
//  "p19", "p6", "p5", "p4", 
//  "p3", "p2", "p18", "p31", 
//  "p32", "p33", "p34", "p35" } 
//;

uint8_t RDS_A3_num = 6;
char *RDS_A3_list[] = {
  "p18", "p6", "p5", 
  "p19", "p31", "p0" }
;
 
//  "p19", "p6", "p5", "p4", 
//  "p3", "p2", "p18", "p31", 
//  "p32", "p33" } 
//;

uint8_t RDS_A4_num = 4;
char *RDS_A4_list[] = {
  "p6", "p5", "p31", "p0" }
;

//  "p19", "p6", "p5", "p4", 
//  "p3", "p18", "p31", "p32" } 
//;

uint8_t RDS_A14_num = 4;
char *RDS_A14_list[] = {
  "p15", "p16", "p28", "p0" }
;


uint8_t RDS_B1 = 0; // RDS_B is used in the L-track. 
uint8_t RDS_B2 = 0;
uint8_t RDS_B3 = 0;
uint8_t RDS_B4 = 0;
uint8_t RDS_B14 = 0;
uint8_t RDS_B1_listnum = 0; // current position in RDS_B1_list
uint8_t RDS_B2_listnum = 0; // current position in RDS_B2_list
uint8_t RDS_B3_listnum = 0; // current position in RDS_B3_list
uint8_t RDS_B4_listnum = 0; // current position in RDS_B4_list
uint8_t RDS_B14_listnum = 0; // current position in RDS_B14_list

uint8_t RDS_B1_num = 12;
char *RDS_B1_list[] = {
  "p24", "p18", "p9", 
  "p6", "p5", "p4", "p11", 
  "p15", "p21", "p25", "p26", "p0" }
;  
//  "p18", "p10", "p6", "p5", 
//  "p4", "p3", "p2", "p1", 
//  "p11", "p17", "p19", "p25", 
//  "p26", "p27", "p28", "p29", "p30" }
//;
uint8_t RDS_B2_num = 8;
char *RDS_B2_list[] = {
  "p18", "p9", "p6", "p5", 
  "p15", "p21", "p25", "p0" }
; 
  //  "p18", "p6", "p5", "p4", 
//  "p3", "p2","p1", "p17", "p25", 
//  "p26", "p27", "p28", "p29","p30" }
//;

uint8_t RDS_B3_num = 6;
char *RDS_B3_list[] = {
  "p18", "p6", "p5", 
  "p15", "p25", "p0" }
; 
//  "p18", "p6", "p5", "p4", 
//  "p3", "p2","p1", "p17", "p25", 
//  "p26", "p27", "p28","p29" }
//;

uint8_t RDS_B4_num = 4;
char *RDS_B4_list[] = {
  "p6", "p5", 
  "p25", "p0" }
; 
//  "p18", "p6", "p5", "p4", 
//  "p3","p2","p1", "p17", 
//  "p25", "p26","p27","p28" }
//;

uint8_t RDS_B14_num = 6;
char *RDS_B14_list[] = {
  "p9", "p10", "p11", 
  "p24", "p23", "p0" }
; 
  
//  "p7", "p8", "p9", "p10", "p11","p12",  
//  "p24", "p23", "p22","p21" }
//;

uint8_t RDS_C1 = 0; // RDS_C is used in the fig8-track. 
uint8_t RDS_C2 = 0;
uint8_t RDS_C3 = 0;
uint8_t RDS_C4 = 0;
uint8_t RDS_C14 = 0;
uint8_t RDS_C1_listnum = 0; // current position in RDS_C1_list
uint8_t RDS_C2_listnum = 0; // current position in RDS_C2_list
uint8_t RDS_C3_listnum = 0; // current position in RDS_C3_list
uint8_t RDS_C4_listnum = 0; // current position in RDS_C4_list
uint8_t RDS_C14_listnum = 0; // current position in RDS_C14_list

uint8_t RDS_C1_num = 8;
char *RDS_C1_list[] = {
  "p1", "p7", "p13", "p14", 
  "p15", "p19", "p24", "p0" }
; 
  
  //  "p1", "p7", "p13", "p14", 
//  "p15", "p16", "p17", "p18", 
//  "p19", "p25" }
//;

uint8_t RDS_C2_num = 6;
char *RDS_C2_list[] = {
  "p19", "p13", "p14",
  "p15", "p24", "p0" }
;

  //  "p1", "p13", "p14", "p15",
//  "p16", "p17", "p18", "p19" }
//;

uint8_t RDS_C3_num = 4;
char *RDS_C3_list[] = {
 "p19", "p13", "p14", "p0" }
;
  //  "p13", "p14", "p15", "p16",
//  "p17", "p18" }
//;

uint8_t RDS_C4_num = 3;
char *RDS_C4_list[] = {
   "p13", "p14", "p0" }
;
//  "p13", "p14", "p15",
//  "p16", "p17" }
//;

uint8_t RDS_C14_num = 3;
char *RDS_C14_list[] = {
   "p25", "p26", "p0" }
;
  //  "p25", "p26", "p27",
//  "p28", "p29" }
//;

uint8_t RDS_D1 = 0; // RDS_D is used in the fig8-track. 
uint8_t RDS_D2 = 0;
uint8_t RDS_D3 = 0;
uint8_t RDS_D4 = 0;
uint8_t RDS_D14 = 0;
uint8_t RDS_D1_listnum = 0; // current position in RDS_D1_list
uint8_t RDS_D2_listnum = 0; // current position in RDS_D2_list
uint8_t RDS_D3_listnum = 0; // current position in RDS_D3_list
uint8_t RDS_D4_listnum = 0; // current position in RDS_D4_list
uint8_t RDS_D14_listnum = 0; // current position in RDS_D14_list

uint8_t RDS_D1_num = 7;
char *RDS_D1_list[] = {
  "p6", "p7", "p8", 
  "p9", "p18", "p24", "p0" }
; 

uint8_t RDS_D2_num = 5;
char *RDS_D2_list[] = {
  "p6", "p7", "p8", "p24", "p0" }
;

uint8_t RDS_D3_num = 4;
char *RDS_D3_list[] = {
  "p7", "p8", "p24", "p0" }
;

uint8_t RDS_D4_num = 3;
char *RDS_D4_list[] = {
   "p7", "p8", "p0" }
;

uint8_t RDS_D14_num = 3;
char *RDS_D14_list[] = {
   "p22", "p23", "p0" }
;



uint8_t RDS_E1 = 0; // RDS_E is used in the teardrop-track. 
uint8_t RDS_E2 = 0;
uint8_t RDS_E3 = 0;
uint8_t RDS_E4 = 0;
uint8_t RDS_E14 = 0;
uint8_t RDS_E1_listnum = 0; // current position in RDS_E1_list - CURRENTLY UNAVAILABLE
uint8_t RDS_E2_listnum = 0; // current position in RDS_E2_list - CURRENTLY UNAVAILABLE
uint8_t RDS_E3_listnum = 0; // current position in RDS_E3_list - CURRENTLY UNAVAILABLE
uint8_t RDS_E4_listnum = 0; // current position in RDS_E4_list
uint8_t RDS_E14_listnum = 0; // current position in RDS_E14_list

uint8_t RDS_E1_num = 7;
char *RDS_E1_list[] = {
  "p1", "p2", "p3", 
  "p9", "p18", "p24", "p0" }
; 

uint8_t RDS_E2_num = 5;
char *RDS_E2_list[] = {
  "p1", "p2", "p3", 
  "p18", "p0" }
; 

uint8_t RDS_E3_num = 4;
char *RDS_E3_list[] = {
  "p1", "p2", 
  "p18", "p0" }
;

uint8_t RDS_E4_num = 3;
char *RDS_E4_list[] = {
  "p1", "p2", "p0" }
; 

uint8_t RDS_E14_num = 3;
char *RDS_E14_list[] = {
   "p36", "p31", "p0" }
;
  
////////////////////////////////////////////////////////////////////////////////////////

//
uint8_t RDS_XZ1 = 0; // RDS_XZ1 is used by Xinyu for the oval track, fixed reward zone. 
uint8_t RDS_XZ1_listnum = 0; // current position in RDS_XZ1_list

uint8_t RDS_XZ1_num = 2;
char *RDS_XZ1_list[] = {
  "p12", "p28"}
; 
//

//
uint8_t RDS_XZ11 = 0; // RDS_XZ11 is used by Xinyu for the oval track, fixed reward zone. Different from RDS_XZ1 for different reward zone. 
uint8_t RDS_XZ11_listnum = 0; // current position in RDS_XZ11__list

uint8_t RDS_XZ11_num = 2;
char *RDS_XZ11_list[] = {
  "p31", "p16"}
;
char *RDS_stim_XZ11[] = {"p4","p17","p0"};
//


//
uint8_t RDS_XZ12 = 0; // RDS_XZ12 is used by Xinyu for the unique maze, but with just one reward
uint8_t RDS_XZ12_listnum = 0; // current position in RDS_XZ12_list

uint8_t RDS_XZ12_num = 2;  
char *RDS_XZ12_list[] = {"p16","p1"} ;
char *RDS_stim_XZ12[] = {"p4","p17","p0"};
//


//
uint8_t RDS_XZ3 = 0; // RDS_XZ3 is used by Xinyu for the triangle track, fixed reward zone. 
uint8_t RDS_XZ3_listnum = 0; // current position in RDS_XZ3_list

uint8_t RDS_XZ3_num = 3;
char *RDS_XZ3_list[] = {
  "p6", "p17", "p30"}
;
//

//
uint8_t RDS_XZ11_TR = 0; // RDS_XZ11_TR is used by Xinyu for the oval track, fixed reward zone.  4 rewards per lap for training.
uint8_t RDS_XZ11_TR_listnum = 0; // current position in RDS_XZ11_TR_list

uint8_t RDS_XZ11_TR_num = 4;
char *RDS_XZ11_TR_list[] = {
  "p31","p23","p16","p7"}
;
char *RDS_stim_XZ11_TR[] = {"p21","p17","p0"};
//
//

//
uint8_t RDS_XZR = 0; // RDS_XZR is used by Xinyu for the oval track, pure random reward.
//See the RDS protocol in the loop for the reward probability
uint8_t RDS_XZR_listnum = 0; // current position in RDS_XZR_list

uint8_t RDS_XZR_num = 0;  //this list is not used for this RDS since it's purely random
char *RDS_XZR_list[] = {} ;
uint8_t th_binomial; //A number from 0 to 100. Controls the reward probability. Larger number means more rewards.
//

// 
long index_RD2[2];
uint8_t RDS_XZRand2 = 0;  //for random reward protocol in which a fixed number of rewards were choosen at the start of each lap. 2 rewards per lap for this one.
uint8_t RDS_XZRand2_listnum = 0; //current reward position
uint8_t RDS_XZRand2_num = 2; //number of rewards
uint8_t RDS_XZRand2_choice_num = 15; //number of choices
char *RDS_XZRand2_list[] = {  //unlike other protocols, this is NOT the actually reward list, but the totoal choices. Reward positions will be assigned at each lap start. 
    "p31","p29","p27","p25","p23","p21","p19","p17","p15","p13","p11","p9","p7","p5","p3"}
;
char *RDS_stim_XZRand2[] = {"p4","p17","p0"};
//

// 
long index_RD3[3];
uint8_t RDS_XZRand3 = 0;  //for random reward protocol in which a fixed number of rewards were choosen at the start of each lap. 3 rewards per lap for this one.
uint8_t RDS_XZRand3_listnum = 0; //current reward position
uint8_t RDS_XZRand3_num = 3; //number of rewards
uint8_t RDS_XZRand3_choice_num = 15; //number of choices
char *RDS_XZRand3_list[] = {  //unlike other protocols, this is NOT the actually reward list, but the totoal choices. Reward positions will be assigned at each lap start. 
    "p31","p29","p27","p25","p23","p21","p19","p17","p15","p13","p11","p9","p7","p5","p3"}
;
char *RDS_stim_XZRand3[] = {"p4","p17","p0"};
//

//
uint8_t RDS_XZ2 = 0; // RDS_XZ2 is used by Xinyu for the symmetric maze
uint8_t RDS_XZ2_listnum = 0; // current position in RDS_XZ2_list

uint8_t RDS_XZ2_num = 2;  
char *RDS_XZ2_list[] = {"p11","p25"} ;
char *RDS_stim_XZ2[] = {"p4","p18","p24"};
//

//
uint8_t RDS_XZTR1 = 0; // RDS_XZTR1 is used by Xinyu for the symmetric maze
uint8_t RDS_XZTR1_listnum = 0; // current position in RDS_XZTR1_list

uint8_t RDS_XZTR1_num = 12;  
char *RDS_XZTR1_list[] = {"p1","p3","p5","p7","p9","p11","p13","p15","p17","p19","p21","p27"} ;  // no reward around the "false alarm" site
//

//
uint8_t RDS_XZTR2 = 0; // RDS_XZTR2 is used by Xinyu for the symmetric maze. Less rewards than RDS_XZTR1
uint8_t RDS_XZTR2_listnum = 0; // current position in RDS_XZTR2_list

uint8_t RDS_XZTR2_num = 7;  
char *RDS_XZTR2_list[] = {"p2","p5","p8","p11","p14","p17","p20"} ;  // no reward around the "false alarm" site
char *RDS_stim_XZTR2[] = {"p4","p18","p24"};
//

//
uint8_t RDS_XZTR3 = 0; // RDS_XZTR3 is used by Xinyu for the symmetric maze. Less rewards than RDS_XZTR2
uint8_t RDS_XZTR3_listnum = 0; // current position in RDS_XZTR3_list

uint8_t RDS_XZTR3_num = 4;  
char *RDS_XZTR3_list[] = {"p1","p11","p18","p26"} ;  // no reward around the "false alarm" site
char *RDS_stim_XZTR3[] = {"p4","p18","p24"};
//
//////////////////////////////////////////////////////////////////////////////////////


uint8_t ASS_A = 0; // tag for Auditory Stimulation Schedule (ASS) version A (ASS_A) for closed loop auditory feedback 
                   // delivered via the "Closed-loop Auditory Stimulation Box" - ("CLASB") 
uint8_t lk_sent = 0; // state for whether or not a lick signal has been sent, 0=not sent, 1=sent
uint8_t pm_sent = 0; // state for whether or not a Protocol Marker (PM) (or Sync pulse) signal has been sent (from Heka), 0=not sent, 1=sent
uint8_t ff_sent = 0; // state for whether or not a Frame Flash signal 
//(FF, photodetector detecting flash on screen) has been sent (from Master-9), 0=not, 1=sent 
uint8_t speed_sent = 0; // state for whether or not a speed threshold is crossed and signal is sent 
//(speed, threshold set in 'if training ==1' code block. If crossed, execute ... , 0=not, 1=sent 
// time captured when a pm interrupt hapens
uint32_t pm_msec;
uint16_t pm_usec;
uint8_t pm_flag = 0; 


// region-dependent intracellular stimulation to external command on Dagan amplifier
//blank for now

// region-dependent intracellular stimulation to gated step command on Dagan amplifier
uint8_t stimZone_flag = 0; // flag for running stimZone, 1=ON, 0=OFF
uint8_t stim_sent = 0; //flag for whether a stimulation was delivered
uint8_t stim_zone_curr = 1; // tag for which stim zone we select 

uint8_t stim_zone_num_1 = 6; // number of items in the stim_zone 1
char *stim_zone_list_1[] = { // list of items that define stim zone 
   "p1", "p2", "p3","p4", "p5", "p6" }
;
uint8_t stim_zone_num_2 = 6; // number of items in the stim_zone 2
char *stim_zone_list_2[] = {
   "p7", "p8", "p9","p10", "p11", "p12" }
;
uint8_t stim_zone_num_3 = 6; // number of items in the stim_zone 3
char *stim_zone_list_3[] = {
   "p13", "p14", "p15","p16", "p17", "p18" }
;
uint8_t stim_zone_num_4 = 6; // number of items in the stim_zone 4
char *stim_zone_list_4[] = {
   "p19", "p20", "p21","p22", "p23", "p24" }
;
uint8_t stim_zone_num_5 = 6; // number of items in the stim_zone 5
char *stim_zone_list_5[] = {
   "p25", "p26", "p27","p28", "p29", "p30" }
;
uint8_t stim_zone_num_6 = 6; // number of items in the stim_zone 6
char *stim_zone_list_6[] = {
   "p31", "p32", "p33","p34", "p35", "p36" }
;

// variables for the stimZone's timer function that gates a TTL oscillator on DIGITAL6
long stim_d = 100000; // duration of pulse in microsec
long stim_i = 200000; // interpulse interval in microsec
long stim_start = 0 ;
long stim_now = 0;
long stim_timer = 0;
uint8_t stim_tag_i = 0; // tag for pulse interval
uint8_t stim_tag_d = 0; // tag for pulse duration




// int tflag; //test code, timer flag


// AD7328 ADC read routine:
// A value is read from the AD7328 using two 16bit SPI transactions
// The first SPI transaction is a write to the control register specifying the channel to read
// The second SPI transaction is the actual analog value read
int16_t adcRead(uint8_t adc, uint8_t coding) {
  // adc = adc input, 0 - 7
  // coding = 0 -> two's compliment
  // coding = 1 -> binary
int16_t adcValue;

  // AD7328 Control Register:
  // 15   14   13   12   11   10   09   08   07   06   05   04   03   02   01   00
  //  W    0    0   ADD2 ADD1 ADD0 M1   M0   PM1  PM0  COD  REF  SEQ1 SEQ2 W/TS  0
  //
  //  W:      Write enable, 1 = write to register, 0 = ignore (read-only)
  //  ADD2-0: Channel
  //  M1-0:   Input mode, 00 -> single ended inputs
  //  PM1-0:  Power management, 00 -> normal mode (all channels powered-up)
  //  COD:    Coding, 0 -> 2's compliment 1 -> straight binary
  //  REF:    Reference select, 1 -> internal reference used
  //  SEQ:    Sequencer control, 00 -> sequencer not used
  //  W/TS:   Weak/Tristate output control, 0 -> output tristate after read transaction

  // AD7328 Read data format:
  // 15   14   13   12   11   10   09   08   07   06   05   04   03   02   01   00
  // ID2  ID1  ID0  D11  D10  D9   D8   D7   D6   D5   D4   D3   D2   D1   D0    x
  //
  // ID2-0: Channel number
  // D11-0: Analog value (D11 is the sign bit if 2's complement coding)

  // For more info on AD7328 see http://www.analog.com/static/imported-files/data_sheets/AD7328.pdf

  // first transaction - select the channel by writing to the control register
  digitalWrite(ADC_PIN, LOW);
  SPI.transfer(0x80 | ((adc & 0x07)<<2));
  SPI.transfer(0x10 | ((coding & 0x01)<<5));
  digitalWrite(ADC_PIN, HIGH);

  // second transaction - read the analog value
  digitalWrite(ADC_PIN, LOW);
  ((uint8_t*)&adcValue)[1] = SPI.transfer(0);
  ((uint8_t*)&adcValue)[0] = SPI.transfer(0);
  digitalWrite(ADC_PIN, HIGH);

  // sign-extend if negative
  if ((coding == 0) && ((adcValue & 0x1000) == 0x1000)) {
    adcValue = (adcValue >> 1) | 0xf000;
  } 
  else {
    adcValue = (adcValue >> 1) & 0x0fff;
  }

  // return the 12 bit value
  return adcValue;
}


////////////
// FUNCTIONS
////////////

// AD5666 DAC write routine:
// An analog output value is written to the AD5666 16-bit DAC in one 32-bit SPI transaction
void dac1Write(uint8_t dac, uint16_t value) {
  // dac = dac output channel, 0 - 3
  // value = 16 bit output value
  //
  // AD5666 register write format:
  // 31  30  29  28  27  26  25  24  23  22  21  20  19  18  17  16  15  14  13  12  11  10  09  08  07  06  05  04  03  02  01  00
  //  x   x   x   x  C3  C2  C1  C0  A3  A2  A1  A0  D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0   x   x   x   x
  //
  // C = Command, 0011 -> write and update dac channel
  // A = Address, 00xx -> write to channel xx where xx = 00(ch A) - 11(ch D), 1111 -> write to all channels
  // D = Data

  // For more info see http://www.analog.com/static/imported-files/data_sheets/AD5666.pdf

  digitalWrite(DAC1_PIN, LOW);
  SPI.transfer(0x03); // CMD = 0011, write & update dac channel
  SPI.transfer(((dac & 0x0f)<<4) | ((value & 0xf000)>>12));
  SPI.transfer((value & 0x0ff0)>>4);
  SPI.transfer((value & 0x000f)<<4);
  digitalWrite(DAC1_PIN, HIGH);
  digitalRead(DAC1_PIN);
}

// AD5754 DAC write routine:
// An analog output value is written to the AD5754 16-bit DAC as a 24-bit SPI transaction
void dac2Write(uint8_t dac, uint16_t value) {
  // dac = dac output channel, 0 - 3
  // value = 16 bit output value

    // AD5724 Register write format:
  // 23   22   21   20   19   18   17   16   15   14   13   12   11   10   09   08   07   06   05   04   03   02   01   00
  //
  // R/W   0   REG2 REG1 REG0 A2   A1   A0   D15  D14  D13  D12  D11  D10  D9   D8   D7   D6   D5   D4   D3   D2   D1   D0
  // R/W:    Read/write, 0 -> write, 1 -> read
  // REG2-0: Register select, 000 -> DAC register, 001 -> output range register, 010 -> power control register, 011 -> control register
  // A2-0:   DAC channel, 0xx -> DAC channel xx where xx = 00(ch A) - 11(ch D), 100 -> all DACs
  // D15-0:  Data

  // For more info see http://www.analog.com/static/imported-files/data_sheets/AD5724_5734_5754.pdf

  digitalWrite(DAC2_PIN, LOW);
  SPI.transfer(dac); // write to DAC register
  SPI.transfer((value & 0xff00)>>8);
  SPI.transfer((value & 0x00ff));
  digitalWrite(DAC2_PIN, HIGH);
  digitalRead(DAC2_PIN);
}


void parse(char *line, char **argv, uint8_t maxArgs) { // parse script
  uint8_t argCount = 0;
  while (*line != '\0') {       /* if not the end of line ....... */

    while (*line == ',' || *line == ' ' || *line == '\t' || *line == '\n') // if one of these chars detected 
      *line++ = '\0';     /* replace commas and white spaces with 0    */
    *argv++ = line;          /* save the argument position     */
    argCount++;
    if (argCount == maxArgs-1)
      break;
    while (*line != '\0' && *line != ',' && *line != ' ' && 
      *line != '\t' && *line != '\n') // if char is not one of these, keep reading
      line++;             /* ie, skip the argument until ...    */
  }
  *argv = line;                 /* mark the end of argument list  */
}

long getValue(char *arg) {
  char input[16];
  char *inp;
  uint8_t cnt;

  inp = input;
  cnt = 0;
  while ((*arg != '\0')&&(cnt < 16)) {
    if (*arg != '.') {
      *inp++ = *arg;
      cnt++;
    }
    *arg++;
  }
  *inp++ ='\0';
  return atol(input);
}

void printTime(uint32_t msec, uint16_t usec) {
  // first print the msec part
  printValue0_U32(msec);
  // next print the usec part
  uint8_t d = 0;
  while (usec > 99) {
    d++;
    usec -= 100;
  }
  uart0_put(d+0x30);
  d = 0;
  while (usec > 9) {
    d++;
    usec -= 10;
  }
  uart0_put(d+0x30);   //0x30 (48) is '0' in the ASCII table
  d = usec;
  uart0_put(d+0x30);
  uart0_put(',');
}

void printTimestamp() {
  uint32_t msec;
  uint16_t usec;
  uint64_t ts;
  // get the time stamp
  ts = getTimestamp();
  msec = (uint32_t)(ts/1000);
  usec = (uint16_t)(ts % 1000);
  printTime(msec, usec);
}

void videoRecON () {
  digitalWrite(DIGITAL4, HIGH);  
}

void videoRecOFF () {
  digitalWrite(DIGITAL4, LOW);  
}

void printRD() {
  digitalWrite(DIGITAL1, HIGH);
  printTimestamp();
  sprintf(stringBuff, "RD"); // indicate a reward event string
  printlnString0((char*)stringBuff);
  RD_cnt++;
  RD_cnt_curr=RD_cnt_curr+100;
  RD_tag = 1;  //the marker meaning that a reward has already been sent. Used to avoid multiple reward for lick-dependent protocol. This tag is reset to 0 by new location event.
  lk_cnt = 0; //reset lick count  
  
  uart1_put(0x80); // row 0, col 0
  Serial1.print(0x0c, BYTE); // clear the display
  sprintf(stringBuff, "RD delivered"); // 
  printString1((char*)stringBuff);
  digitalWrite(DIGITAL1, LOW);
}

void printEN() {
  digitalWrite(DIGITAL1, HIGH);
  printTimestamp();
  sprintf(stringBuff, "EN");
  printlnString0((char*)stringBuff);
  digitalWrite(DIGITAL1, LOW);
}

// AP detection using hardware based AP-detection box and the interrupt pins
void th1() { // ascending phase trigger, first AP tag
  if (th2Detected == 0) { // if high threshold is also detected, cancel tag
    th1Detected = 1;
    startTimer(th1Timer, DELAY_TH1, th1TimerCB, 0); // if high threshold is triggered before timer finishes, cancel 
  }
}

void th2() { // high threshold trigger, define as noise and ignore ascending trigger tag.
  th2Detected = 1;
  startTimer(th2Timer, DELAY_TH2, th2TimerCB, 0);
  if (th1Detected == 1) {
    stopTimer(th1Timer);
    th1Detected = 0;
  }
}

void th3() { // descending phase trigger, second AP tag
  if (th1Detected == 1) { // if first AP tag
    thDetect = 1; // set final AP tag - will now output a "timestamp,TH" marker string
    th1Detected = 0; // reset fist AP tag
    stopTimer(th1Timer);
  }
}

// reset th1Timer
void th1TimerCB(uint8_t arg) {
  th1Detected = 0;
}
// reset th2Timer
void th2TimerCB(uint8_t arg) {
  th2Detected = 0;
}

// the PM or sync pulses could be detected on an interrupt pin. but it not currently used
void PM() {
  // get the time stamp
  uint64_t ts;
  // get the time stamp
  ts = getTimestamp();
  pm_msec = (uint32_t)(ts/1000);
  pm_usec = (uint16_t)(ts % 1000);
  pm_flag = 1;
}


void lapMarker() {
  lap_cnt++;
  lap_tag=0;
  printTimestamp();
  sprintf(stringBuff, "lapMarker,");
  printString0((char*)stringBuff);
  printValue0_U32(0);
  printNewline0();
  
  stim_sent=0; //reset the flag so a new stim can be delivered


  if (randRD_flag) {  //random reward protocol being used
    
    if (RDS_XZRand2) {
    
      for (ii=0;ii<RDS_XZRand2_num;ii++) {
        index_RD2[ii]=random(RDS_XZRand2_choice_num);  //generate a random integer from 0 to choice_num-1;
        for (jj=0;jj<ii;jj++) {  //search for all previously generateed locations;
          if (index_RD2[ii] == index_RD2[jj]) {
            index_RD2[ii]++;  //move to the next one if the current location coincides with any previous ones.
            if (index_RD2[ii] > (RDS_XZRand2_choice_num-1)) {
              index_RD2[ii] = 0;        //loop back if the index exceeds the range
            }
          }
        }      
      }
      
      for (ii=0;ii<RDS_XZRand2_num;ii++) {      //sorting
        for (jj=(ii+1);jj<(RDS_XZRand2_num-1);jj++) {
          if (index_RD2[jj] < index_RD2[ii]) {
            temp = index_RD2[ii];
            index_RD2[ii] = index_RD2[jj];
            index_RD2[jj] = temp;
          } 
        }
      }
      
    }
    
    else if (RDS_XZRand3) {
      
      for (ii=0;ii<RDS_XZRand3_num;ii++) {
        index_RD2[ii]=random(RDS_XZRand3_choice_num);  //generate a random integer from 0 to choice_num-1;
        for (jj=0;jj<ii;jj++) {  //search for all previously generateed locations;
          if (index_RD3[ii] == index_RD3[jj]) {
            index_RD3[ii]++;  //move to the next one if the current location coincides with any previous ones.
            if (index_RD3[ii] > (RDS_XZRand3_choice_num-1)) {
              index_RD3[ii] = 0;        //loop back if the index exceeds the range
            }
          }
        }      
      }
      
      for (ii=0;ii<RDS_XZRand3_num;ii++) {      //sorting
        for (jj=(ii+1);jj<(RDS_XZRand3_num-1);jj++) {
          if (index_RD3[jj] < index_RD3[ii]) {
            temp = index_RD3[ii];
            index_RD3[ii] = index_RD3[jj];
            index_RD3[jj] = temp;
          } 
        }
      }
      
    }
      
  }
  
}

// random reward schedule. picks a number from the total list (0-36). 
// if current location = picked location, RD is delivered
// loop will "roll the dice" rollMax times.
// if RD is delivered, rolling is stopped for the loop
void randRD_roll() { 
  if (randRD_roll_tag) {
    while (rollNum <= rollMax) {
      randNum = random(RDS_x0_num);
      if  ((strcmp(eventstr_1,&RDS_x0_list[randNum][0]) == 0)) {
        printRD();
        rollNum = 1; //?
        randRD_roll_tag = 0; // received RD, turn off randRoll sequence
      }
      rollNum++;
    }
    randRD_roll_tag = 0;
    rollNum=1;
  }
}

// New random protocol based on binomial distribution
void randRD_binomial() {
  randNum=random(100);
  if (randNum<=th_binomial) {  //generate a random number from 0 to 100. Give the reward when it's smaller than th_binomial. So th_binomial controls the probability.
    printRD();
  }
}

// New random protocol with pre-chosen RD locations at the beginning of each lap
// see lapMarker for generating the locations

/*
void randRD_fixedNum() {
  uint8_t i;
  uint8_t flag_match = 0;
  for(i=0;i<num_randRD;i++) {
    if (strcmp(eventstr_1,&RDS_x0_list[index_RD[i]][0]) == 0) {
      flag_match=1;
    }
  }
  if (flag_match) {
    printRD();
  }
}
*/

void stimZone() {  //new stim protocol. just send out a TTL for the single stim location. use the gate mode in the amplifier

  if (!stim_sent) {   //stim_sent is a flag indicating that a stimulation has been injected. it'll be reset to 0 at the lapMarker  
    
    if (RDS_XZ2) {  
    
      if ((strcmp(eventstr_1,&RDS_stim_XZ2[stim_zone_curr-1][0]) == 0) ||   //if any event string is the position that is the stim zone
          (strcmp(eventstr_2,&RDS_stim_XZ2[stim_zone_curr-1][0]) == 0) ||
          (strcmp(eventstr_3,&RDS_stim_XZ2[stim_zone_curr-1][0]) == 0) ||              
          (strcmp(eventstr_4,&RDS_stim_XZ2[stim_zone_curr-1][0]) == 0)) {
  
          digitalWrite(DIGITAL6, HIGH);
          stim_sent=1;
    
          printTimestamp();
          sprintf(stringBuff, "Stim"); // indicate a reward event string
          printlnString0((char*)stringBuff);
    
          uart1_put(0x80); // row 0, col 0
          Serial1.print(0x0c, BYTE); // clear the display
          sprintf(stringBuff, "Stim"); // 
          printString1((char*)stringBuff);
    
          digitalWrite(DIGITAL6, LOW);    
      }      
    }
    
    if (RDS_XZ12) {  
    
      if ((strcmp(eventstr_1,&RDS_stim_XZ12[stim_zone_curr-1][0]) == 0) ||   //if any event string is the position that is the stim zone
          (strcmp(eventstr_2,&RDS_stim_XZ12[stim_zone_curr-1][0]) == 0) ||
          (strcmp(eventstr_3,&RDS_stim_XZ12[stim_zone_curr-1][0]) == 0) ||              
          (strcmp(eventstr_4,&RDS_stim_XZ12[stim_zone_curr-1][0]) == 0)) {
  
          digitalWrite(DIGITAL6, HIGH);
          stim_sent=1;
    
          printTimestamp();
          sprintf(stringBuff, "Stim"); // indicate a reward event string
          printlnString0((char*)stringBuff);
    
          uart1_put(0x80); // row 0, col 0
          Serial1.print(0x0c, BYTE); // clear the display
          sprintf(stringBuff, "Stim"); // 
          printString1((char*)stringBuff);
    
          digitalWrite(DIGITAL6, LOW);    
      }      
    }
    
    
    else if (RDS_XZ11) {
      if ((strcmp(eventstr_1,&RDS_stim_XZ11[stim_zone_curr-1][0]) == 0) ||   //if any event string is the position that is the stim zone
          (strcmp(eventstr_2,&RDS_stim_XZ11[stim_zone_curr-1][0]) == 0) ||
          (strcmp(eventstr_3,&RDS_stim_XZ11[stim_zone_curr-1][0]) == 0) ||              
          (strcmp(eventstr_4,&RDS_stim_XZ11[stim_zone_curr-1][0]) == 0)) {
  
          digitalWrite(DIGITAL6, HIGH);
          stim_sent=1;
    
          printTimestamp();
          sprintf(stringBuff, "Stim"); // indicate a reward event string
          printlnString0((char*)stringBuff);
    
          uart1_put(0x80); // row 0, col 0
          Serial1.print(0x0c, BYTE); // clear the display
          sprintf(stringBuff, "Stim"); // 
          printString1((char*)stringBuff);
    
          digitalWrite(DIGITAL6, LOW);    
      }
    }
    
    else if (RDS_XZ11_TR) {
      if ((strcmp(eventstr_1,&RDS_stim_XZ11_TR[stim_zone_curr-1][0]) == 0) ||   //if any event string is the position that is the stim zone
          (strcmp(eventstr_2,&RDS_stim_XZ11_TR[stim_zone_curr-1][0]) == 0) ||
          (strcmp(eventstr_3,&RDS_stim_XZ11_TR[stim_zone_curr-1][0]) == 0) ||              
          (strcmp(eventstr_4,&RDS_stim_XZ11_TR[stim_zone_curr-1][0]) == 0)) {
  
          digitalWrite(DIGITAL6, HIGH);
          stim_sent=1;
    
          printTimestamp();
          sprintf(stringBuff, "Stim"); // indicate a reward event string
          printlnString0((char*)stringBuff);
    
          uart1_put(0x80); // row 0, col 0
          Serial1.print(0x0c, BYTE); // clear the display
          sprintf(stringBuff, "Stim"); // 
          printString1((char*)stringBuff);
    
          digitalWrite(DIGITAL6, LOW);    
      }
    }
    
    else if (RDS_XZTR2) {
      if ((strcmp(eventstr_1,&RDS_stim_XZTR2[stim_zone_curr-1][0]) == 0) ||   //if any event string is the position that is the stim zone
          (strcmp(eventstr_2,&RDS_stim_XZTR2[stim_zone_curr-1][0]) == 0) ||
          (strcmp(eventstr_3,&RDS_stim_XZTR2[stim_zone_curr-1][0]) == 0) ||              
          (strcmp(eventstr_4,&RDS_stim_XZTR2[stim_zone_curr-1][0]) == 0)) {
  
          digitalWrite(DIGITAL6, HIGH);
          stim_sent=1;
    
          printTimestamp();
          sprintf(stringBuff, "Stim"); // indicate a reward event string
          printlnString0((char*)stringBuff);
    
          uart1_put(0x80); // row 0, col 0
          Serial1.print(0x0c, BYTE); // clear the display
          sprintf(stringBuff, "Stim"); // 
          printString1((char*)stringBuff);
    
          digitalWrite(DIGITAL6, LOW);
      }
    }
    
    else if (RDS_XZTR3) {
      if ((strcmp(eventstr_1,&RDS_stim_XZTR3[stim_zone_curr-1][0]) == 0) ||   //if any event string is the position that is the stim zone
          (strcmp(eventstr_2,&RDS_stim_XZTR3[stim_zone_curr-1][0]) == 0) ||
          (strcmp(eventstr_3,&RDS_stim_XZTR3[stim_zone_curr-1][0]) == 0) ||              
          (strcmp(eventstr_4,&RDS_stim_XZTR3[stim_zone_curr-1][0]) == 0)) {
  
          digitalWrite(DIGITAL6, HIGH);
          stim_sent=1;
    
          printTimestamp();
          sprintf(stringBuff, "Stim"); // indicate a reward event string
          printlnString0((char*)stringBuff);
    
          uart1_put(0x80); // row 0, col 0
          Serial1.print(0x0c, BYTE); // clear the display
          sprintf(stringBuff, "Stim"); // 
          printString1((char*)stringBuff);
    
          digitalWrite(DIGITAL6, LOW);
      }
    }
    
    else if (RDS_XZRand2) {
      if ((strcmp(eventstr_1,&RDS_stim_XZRand2[stim_zone_curr-1][0]) == 0) ||   //if any event string is the position that is the stim zone
          (strcmp(eventstr_2,&RDS_stim_XZRand2[stim_zone_curr-1][0]) == 0) ||
          (strcmp(eventstr_3,&RDS_stim_XZRand2[stim_zone_curr-1][0]) == 0) ||              
          (strcmp(eventstr_4,&RDS_stim_XZRand2[stim_zone_curr-1][0]) == 0)) {
  
          digitalWrite(DIGITAL6, HIGH);
          stim_sent=1;
    
          printTimestamp();
          sprintf(stringBuff, "Stim"); // indicate a reward event string
          printlnString0((char*)stringBuff);
    
          uart1_put(0x80); // row 0, col 0
          Serial1.print(0x0c, BYTE); // clear the display
          sprintf(stringBuff, "Stim"); // 
          printString1((char*)stringBuff);
    
          digitalWrite(DIGITAL6, LOW);    
      }
    }
    
    else if (RDS_XZRand3) {
      if ((strcmp(eventstr_1,&RDS_stim_XZRand3[stim_zone_curr-1][0]) == 0) ||   //if any event string is the position that is the stim zone
          (strcmp(eventstr_2,&RDS_stim_XZRand3[stim_zone_curr-1][0]) == 0) ||
          (strcmp(eventstr_3,&RDS_stim_XZRand3[stim_zone_curr-1][0]) == 0) ||              
          (strcmp(eventstr_4,&RDS_stim_XZRand3[stim_zone_curr-1][0]) == 0)) {
  
          digitalWrite(DIGITAL6, HIGH);
          stim_sent=1;
    
          printTimestamp();
          sprintf(stringBuff, "Stim"); // indicate a reward event string
          printlnString0((char*)stringBuff);
    
          uart1_put(0x80); // row 0, col 0
          Serial1.print(0x0c, BYTE); // clear the display
          sprintf(stringBuff, "Stim"); // 
          printString1((char*)stringBuff);
    
          digitalWrite(DIGITAL6, LOW);    
      }
    }
    
  }
  
}


/*
// stimZone will output a TTL when animal is in contact with 1 of 6 photobeams in VR
// there are 6 stim zones, each containing 6 events (photombeams). spatial range is ~20-30cm
void stimZone() {
  stim_tag_i=0; // turn OFF stimZone_timer() main gate
  digitalWrite(DIGITAL6, LOW); // maintain default state, stim OFF
  uart1_put(0x80); // row 0, col 0
  Serial1.print(0x0c, BYTE); // clear the display

  if (stim_zone_curr == 1) {  
    for (uint8_t i = 0; i < stim_zone_num_1; i++) { // loop through list
      if  ((strcmp(eventstr_1,&stim_zone_list_1[i][0]) == 0)) { // if get a hit
        //digitalWrite(DIGITAL6, HIGH); // stim TTL on
        stim_tag_i=1; // turn on stimZone_timer's main gate 
        sprintf(stringBuff, "inside stim zone 1"); // 
        printString1((char*)stringBuff);
      }
    }
  }    
  else if (stim_zone_curr == 2) {  
    for (uint8_t i = 0; i < stim_zone_num_2; i++) { // loop through list
      if  ((strcmp(eventstr_1,&stim_zone_list_2[i][0]) == 0)) { // if get a hit
        //digitalWrite(DIGITAL6,HIGH); // stim TTL on
        stim_tag_i=1; // turn on stimZone_timer's main gate
        sprintf(stringBuff, "inside stim zone 2"); // 
        printString1((char*)stringBuff);
      }
    }
  }
  else if (stim_zone_curr == 3) {  
    for (uint8_t i = 0; i < stim_zone_num_3; i++) { // loop through list
      if  ((strcmp(eventstr_1,&stim_zone_list_3[i][0]) == 0)) { // if get a hit
        //digitalWrite(DIGITAL6,HIGH); // stim TTL on
        stim_tag_i=1; // turn on stimZone_timer's main gate
        sprintf(stringBuff, "inside stim zone 3"); // 
        printString1((char*)stringBuff);
      }
    }
  }
  else if (stim_zone_curr == 4) {  
    for (uint8_t i = 0; i < stim_zone_num_4; i++) { // loop through list
      if  ((strcmp(eventstr_1,&stim_zone_list_4[i][0]) == 0)) { // if get a hit
        //digitalWrite(DIGITAL6,HIGH); // stim TTL on
        stim_tag_i=1; // turn on stimZone_timer's main gate
        sprintf(stringBuff, "inside stim zone 4"); // 
        printString1((char*)stringBuff);
      }
    }
  }
  else if (stim_zone_curr == 5) {  
    for (uint8_t i = 0; i < stim_zone_num_5; i++) { // loop through list
      if  ((strcmp(eventstr_1,&stim_zone_list_5[i][0]) == 0)) { // if get a hit
        //digitalWrite(DIGITAL6,HIGH); // stim TTL on
        stim_tag_i=1; // turn on stimZone_timer's main gate
        sprintf(stringBuff, "inside stim zone 5"); // 
        printString1((char*)stringBuff);
      }
    }
  }
  else if (stim_zone_curr == 6) {  
    for (uint8_t i = 0; i < stim_zone_num_6; i++) { // loop through list
      if  ((strcmp(eventstr_1,&stim_zone_list_6[i][0]) == 0)) { // if get a hit
        //digitalWrite(DIGITAL6,HIGH); // stim TTL on
        stim_tag_i=1; // turn on stimZone_timer's main gate
        sprintf(stringBuff, "inside stim zone 6"); // 
        printString1((char*)stringBuff);
      }
    }
  }
}

*/

// Servo motor control routines
void servoCommand(uint8_t device, uint8_t command, long arg) {
  uart1_put(0x80); // row 0, col 0
  Serial1.print(0x0c, BYTE); // clear the display
  sprintf(stringBuff, "SERVO MOVED"); // 
  printString1((char*)stringBuff);
  uart3_put(device);
  uart3_put(command);
  uart3_put(arg&0xff);
  uart3_put((arg>>8)&0xff);
  uart3_put((arg>>16)&0xff);
  uart3_put(arg>>24);
}

void servoStop(uint8_t device) {
  uart1_put(0x80); // row 0, col 0
  Serial1.print(0x0c, BYTE); // clear the display
  sprintf(stringBuff, "SERVO STOPPED"); // 
  printString1((char*)stringBuff);
  uart3_put(device);
  uart3_put(23);
  uart3_put(0);
  uart3_put(0);
  uart3_put(0);
  uart3_put(0);
}

void servoRenumber() {
  uart1_put(0x80); // row 0, col 0
  Serial1.print(0x0c, BYTE); // clear the display
  sprintf(stringBuff, "SERVOS RENUMBERED"); // 
  printString1((char*)stringBuff);
  uart3_put(0);
  uart3_put(2);
  uart3_put(0);
  uart3_put(0);
  uart3_put(0);
  uart3_put(0);
}

void servoReset(uint8_t device) {
  uart1_put(0x80); // row 0, col 0
  Serial1.print(0x0c, BYTE); // clear the display
  sprintf(stringBuff, "SERVOS RESET"); // 
  printString1((char*)stringBuff);
  uart3_put(device);
  uart3_put(0);
  uart3_put(0);
  uart3_put(0);
  uart3_put(0);
  uart3_put(0);
}

void servoHome(uint8_t device) {
  uart1_put(0x80); // row 0, col 0
  Serial1.print(0x0c, BYTE); // clear the display
  sprintf(stringBuff, "SERVO HOME"); // indicate a reward event string
  printString1((char*)stringBuff);
  uart3_put(device);
  uart3_put(HOME_CMD);
  uart3_put(0);
  uart3_put(0);
  uart3_put(0);
  uart3_put(0);
}

void servoEnd(uint8_t device) {
  uart1_put(0x80); // row 0, col 0
  Serial1.print(0x0c, BYTE); // clear the display
  sprintf(stringBuff, "SERVO END"); // 
  printString1((char*)stringBuff);
  uart3_put(device);
  uart3_put(MOVE_ABSOLUTE_CMD);
  uart3_put(ENDPOS&0xff);
  uart3_put((ENDPOS>>8)&0xff);
  uart3_put((ENDPOS>>16)&0xff);
  uart3_put(ENDPOS>>24);
}

void setServoSpeed(uint8_t device, long arg) {
  uart1_put(0x80); // row 0, col 0
  Serial1.print(0x0c, BYTE); // clear the display
  sprintf(stringBuff, "SET SERVO SPEED"); // 
  printString1((char*)stringBuff);
  uart3_put(device);
  uart3_put(SET_SPEED_CMD);
  uart3_put(arg&0xff);
  uart3_put((arg>>8)&0xff);
  uart3_put((arg>>16)&0xff);
  uart3_put(arg>>24);
}

void setMaxPos(uint8_t device, long arg) {
  uart1_put(0x80); // row 0, col 0
  Serial1.print(0x0c, BYTE); // clear the display
  sprintf(stringBuff, "SET SERVO MAX"); // 
  printString1((char*)stringBuff);
  uart3_put(device);
  uart3_put(SET_MAX_POS_CMD);
  uart3_put(arg&0xff);
  uart3_put((arg>>8)&0xff);
  uart3_put((arg>>16)&0xff);
  uart3_put(arg>>24);
}

void setMicrostepRes(uint8_t device, long arg) {
  uart1_put(0x80); // row 0, col 0
  Serial1.print(0x0c, BYTE); // clear the display
  sprintf(stringBuff, "SET MICROSTEP RES"); // 
  printString1((char*)stringBuff);
  uart3_put(device);
  uart3_put(SET_MICROSTEP_RES_CMD);
  uart3_put(arg&0xff);
  uart3_put((arg>>8)&0xff);
  uart3_put((arg>>16)&0xff);
  uart3_put(arg>>24);
}




void setup() {
  bcs.begin();    // !!! first, initialize the BCS   
  // and then the panels 
  adc.begin();    // all ADC channels on, set to +/-10V range 
  dac.begin();    // all DAC channels on, set to +/-10V range
  lcd.begin();    // lcd monitor
  driver.begin(); // initialize the driver module
      
  // this application uses +/-5V input and output ranges 
  for( uint8_t i = 0; i < 8; i++)
  {
    if( i < 4 ) dac.setRange(i, BIP5);  // only 4 dacs
    adc.setRange(i, BIP5);   // but 8 adcs
  }
     
  pinMode(LED_PIN, OUTPUT); // Green LED on the front
  
//  pinMode(SOLENOID1, OUTPUT);
//  pinMode(SOLENOID2, OUTPUT);
//  pinMode(SOLENOID3, OUTPUT);
//  pinMode(SOLENOID4, OUTPUT);
//  pinMode(SOLENOID5, OUTPUT);
//  pinMode(SOLENOID6, OUTPUT);
//  pinMode(SOLENOID7, OUTPUT);
//  pinMode(SOLENOID8, OUTPUT);
//  pinMode(ADC_PIN, OUTPUT);
//  pinMode(DAC2_PIN, OUTPUT); // used for the closed-loop auditory stimulation box  
//  pinMode(DIGITAL1, OUTPUT); // DIGITAL1 is the reward (RD) signal
//  pinMode(DIGITAL2, INPUT);  // Frame Flashes (FF) signal to the cameras.  
//  pinMode(DIGITAL3, INPUT);  // Protocol Markers (PM) (or SYNC 'S' pulses from Heka)
//  pinMode(DIGITAL4, OUTPUT); // Start/Stop video recording signal, HIGH=videoRecON, LOW=videoRecOFF
//  pinMode(DIGITAL5, INPUT);  // Licks (LK)
//  pinMode(DIGITAL6, OUTPUT);  // TTL to step command gate on Dagan amplifier

  bcs.setIO( 1, DIGITAL1, OUTPUT); // pinMode(DIGITAL1, OUTPUT); // DIGITAL1 is the reward (RD) signal
//bcs.setIO( 2, DIGITAL2, INPUT); // pinMode(DIGITAL2, INPUT);  // Frame Flashes (FF) signal to the cameras.
  bcs.setIO( 2, DIGITAL2, OUTPUT); // pinMode(DIGITAL2, OUTPUT);  // TTL to step command gate on ScanIMage Next File Trigger.  
  bcs.setIO( 3, DIGITAL3, INPUT); // pinMode(DIGITAL3, INPUT);  // Protocol Markers (PM) (or SYNC 'S' pulses from Heka)
  bcs.setIO( 4, DIGITAL4, OUTPUT);// pinMode(DIGITAL4, OUTPUT); // Start/Stop video recording signal, HIGH=videoRecON, LOW=videoRecOFF
  bcs.setIO( 5, DIGITAL5, INPUT);// pinMode(DIGITAL5, INPUT);  // Licks (LK)
  bcs.setIO( 6, DIGITAL6, OUTPUT);// pinMode(DIGITAL6, OUTPUT);  // TTL to step command gate on Dagan amplifier
  // added line to control ScanIMage, Master9 // JS
  bcs.setIO( 7, DIGITAL7, OUTPUT);// pinMode(DIGITAL7, OUTPUT);  // TTL to step command gate on ScanIMage Start/End 
  bcs.setIO( 8, DIGITAL8, OUTPUT);// pinMode(DIGITAL8, OUTPUT);  // TTL to Master9

  digitalWrite(LED_PIN, LOW);
  
  digitalWrite(DIGITAL1, LOW);
  digitalWrite(DIGITAL2, LOW);
  digitalWrite(DIGITAL4, LOW);
  digitalWrite(DIGITAL6, LOW);
  digitalWrite(DIGITAL7, LOW);
  digitalWrite(DIGITAL8, LOW);
  
//  digitalWrite(SOLENOID1, LOW);
//  digitalWrite(SOLENOID2, LOW);
//  digitalWrite(SOLENOID3, LOW);
//  digitalWrite(SOLENOID4, LOW);
//  digitalWrite(SOLENOID5, LOW);
//  digitalWrite(SOLENOID6, LOW);
//  digitalWrite(SOLENOID7, LOW);
//  digitalWrite(SOLENOID8, LOW);
//  digitalWrite(ADC_PIN, HIGH);
//  digitalWrite(DAC2_PIN, HIGH);
//  digitalWrite(DIGITAL1, LOW);
//  digitalWrite(DIGITAL4, LOW);
//  digitalWrite(DIGITAL6, LOW);

//  // initialize SPI:  
//  SPI.begin(); 
//  // use SPI clock mode 2
//  SPI.setDataMode(SPI_MODE2);
//  // set clock mode to FCLK/4
//  SPI.setClockDivider(SPI_CLOCK_DIV4);

  // set dac2 outputs to 0V

  dac.write(0, 0x0000);  // dac2Write(0, 0);
  dac.write(1, 0x0000);  // dac2Write(1, 0);
  dac.write(2, 0x0000);  // dac2Write(2, 0);
  dac.write(3, 0x0000);  // dac2Write(3, 0);

  driver.setChannelOff(1); // turn OFF driver channel, close solenoid
  driver.setChannelOff(2); // turn OFF driver channel, 
  driver.setChannelOff(3); // turn OFF driver channel, 
  driver.setChannelOff(4); // turn OFF driver channel, 
  driver.setChannelOff(5); // turn OFF driver channel, 
  driver.setChannelOff(6); // turn OFF driver channel, 
  driver.setChannelOff(7); // turn OFF driver channel, 
  driver.setChannelOff(8); // turn OFF driver channel, 


  /////////////////////
  // DAC1 (AD5666) setup
  /////////////////////
  
  // AD5666 Reference register write format:
  // 31  30  29  28  27  26  25  24  23  22  21  20  19  18  17  16  15  14  13  12  11  10  09  08  07  06  05  04  03  02  01  00
  //  x   x   x   x  C3  C2  C1  C0   x   x   x   x   x   x   x   x   x   x   x   x   x   x   x   x   x   x   x   x   x   x  D1  D0

  // C3-0:   Command, 1000 -> write to reference register
  // D1-0:   Ref setting, 00 -> Stand-alone with REF off, 01 -> Stand-alone with REF on, 10 -> Daisy-chain with REF off, 
  //         11 -> Daisy-chain with REF on

  // AD5666 Power control register write format:
  // 31  30  29  28  27  26  25  24  23  22  21  20  19  18  17  16  15  14  13  12  11  10  09  08  07  06  05  04  03  02  01  00
  //  x   x   x   x  C3  C2  C1  C0   x   x   x   x   x   x   x   x   x   x   x   x   x   x  PD1 PD0  x   x   x   x   D   C   B   A

  // C:       Command, 0100 -> write to power control register
  // PD1,PD0: 00 -> normal mode, 01 -> power down with 1kOhm to gnd, 10 -> power down with 100kOhm to gnd, 
  //          11 -> power down with tristate output
  // A,B,C,D: Channel selection bits, 1 -> channel written

  // For more info see http://www.analog.com/static/imported-files/data_sheets/AD5666.pdf

  
  
//  
//  
//  // Setup DAC REF register
//  digitalWrite(DAC1_PIN, LOW);
//  SPI.transfer(0x08); // CMD = 1000
//  SPI.transfer(0x00);
//  SPI.transfer(0x00);
//  SPI.transfer(0x01); // Standalone mode, REF on
//  digitalWrite(DAC1_PIN, HIGH);
//  digitalRead(DAC1_PIN); // add some time
//
//  // Power up all four DACs
//  digitalWrite(DAC1_PIN, LOW);
//  SPI.transfer(0x04); // CMD = 0100
//  SPI.transfer(0x00);
//  SPI.transfer(0x00); // Normal operation (power-on)
//  SPI.transfer(0x0f); // All four DACs
//  digitalWrite(DAC1_PIN, HIGH);
//  digitalRead(DAC1_PIN);
//
//  // Set dac1 outputs to 0V
//  dac1Write(0, 0x0000); // 0V
//  dac1Write(1, 0x0000); // 0V
//  dac1Write(2, 0x0000); // 0V
//  dac1Write(3, 0x0000); // 0V
//
//  
//  
//  /////////////////////
//  // ADC (AD7328) setup:
//  /////////////////////
//  
//  // AD7328 Control Register:
//  // 15   14   13   12   11   10   09   08   07   06   05   04   03   02   01   00
//  //  W    0    0   ADD2 ADD1 ADD0 M1   M0   PM1  PM0  COD  REF  SEQ1 SEQ2 W/TS  0
//
//  //  W:      Write enable, 1 = write to register, 0 = ignore (read-only)
//  //  ADD2-0: Channel
//  //  M1-0:   Input mode, 00 -> single ended inputs
//  //  PM1-0:  Power management, 00 -> normal mode (all channels powered-up)
//  //  COD:    Coding, 0 -> 2's compliment 1 -> straight binary
//  //  REF:    Reference select, 1 -> internal reference used
//  //  SEQ:    Sequencer control, 00 -> sequencer not used
//  //  W/TS:   Weak/Tristate output control, 0 -> output tristate after read transaction
//
//  // AD7328 Range Register 1:
//  // 15   14   13   12   11   10   09   08   07   06   05   04   03   02   01   00
//  //  1    0    1   0A   0B   1A   1B   2A   2B   3A   3B    0    0    0    0    0
//  // xA, xB: Channel x range setting, 00 -> +/-10V, 01 -> +/-5V, 10 -> +/-2.5V, 11 -> 0V - 10V
//
//  // AD7328 Range Register 2:
//  // 15   14   13   12   11   10   09   08   07   06   05   04   03   02   01   00
//  //  1    1    0   4A   4B   5A   5B   6A   6B   7A   7B    0    0    0    0    0
//  // xA, xB: Channel x range setting, 00 -> +/-10V, 01 -> +/-5V, 10 -> +/-2.5V, 11 -> 0V - 10V
//
//  // AD7328 Sequence Register:
//  // 15   14   13   12   11   10   09   08   07   06   05   04   03   02   01   00
//  //  1    1    1   Ch0  Ch1  Ch2  Ch3  Ch4  Ch5  Ch6  Ch7   0    0    0    0    0
//  // Ch0-7:  1 -> channel is included is sequencer operation
//
//  // write range register 1: +/- 5V range on ch0 0,1,2,3
//  // 0xa000 -> +/- 10V range on ch 0,1,2,3
//  // 0xaaa0 -> +/- 5V range on ch 0,1,2,3
//  // 0xb540 -> +/- 2.5V range on ch 0,1,2,3
//  // 0xbfe0 -> 0 - 10V range on ch 0,1,2,3
//  digitalWrite(ADC_PIN, LOW);
//  SPI.transfer(0xaa);
//  SPI.transfer(0xa0);
//  digitalWrite(ADC_PIN, HIGH);
//
//  // write range register 2: +/-5V range on ch 4,5,6,7
//  // 0xc000 -> +/- 10V range on ch 4,5,6,7
//  // 0xcaa0 -> +/- 5V range on ch 4,5,6,7
//  // 0xd540 -> +/- 2.5V range on ch 4,5,6,7
//  // 0xdfe0 -> 0 - 10V range on ch 4,5,6,7
//  digitalWrite(ADC_PIN, LOW);
//  SPI.transfer(0xca);
//  SPI.transfer(0xa0);
//  digitalWrite(ADC_PIN, HIGH);
//
//  // write sequence register: all sequence bits off
//  digitalWrite(ADC_PIN, LOW);
//  SPI.transfer(0xe0);
//  SPI.transfer(0x00);
//  digitalWrite(ADC_PIN, HIGH);
//
//  // control register: Ch 000, Mode = 00, PM = 00, Code = 0, Ref = 1, Seq = 00, W/TS = 0
//  digitalWrite(ADC_PIN, LOW);
//  SPI.transfer(0x80);
//  SPI.transfer(0x10);
//  digitalWrite(ADC_PIN, HIGH);
//
//
//  /////////////////////
//  // DAC2 (AD5754) setup
//  /////////////////////
//  
//  // AD5724 Power Control Register write format:
//  // 23   22   21   20   19   18   17   16   15   14   13   12   11   10   09   08   07   06   05   04   03   02   01   00
//  //  0   0    0    1     0    0    0    0    x    x    x    x    x   OCD  OCC  OCB  OCA   0   TSD   0   PUD  PUC  PUB  PUA
//  //
//  // OCx:   Overcurrent alert for DAC x
//  // TSD:   Thermal shutdown alert
//  // PUx:   Power-up DAC x
//
//  // AD5724 Control Register write format:
//  // 23   22   21   20   19   18   17   16   15   14   13   12   11   10   09   08   07   06   05   04   03   02   01   00
//  //  0   0    0    1     1   CMD2 CMD1 CMD0  x    x    x    x    x    x    x    x    x    x    x    x   D3   D2   D1   D0
//  //
//  // CMD:   000 -> nop, 001 -> control cmd, 100 -> clear DACs, 101 -> load DACs  
//  // D3-0:  Data for control cmd, D0 = SDO disable, D1 = CLR select, D2 = Clamp enable, D3 = TSD enable
//
//  // AD5724 Range Register write format:
//  // 23   22   21   20   19   18   17   16   15   14   13   12   11   10   09   08   07   06   05   04   03   02   01   00
//  //  0   0    0    0     1   DAC2 DAC1 DAC0  x    x    x    x    x    x    x    x    x    x    x    x    x   R2   R1   R0
//  //
//  // DAC:   DAC select, 000 -> DACA, 001 -> DACB, 010 -> DACC, 011 -> DACD, 100 -> all four DACs  
//  // R3-0:  Range select, 000 -> 0-5V, 001 -> 0-10V, 010 -> 0-10.8V, 011 -> +/-5V, 100 -> +/-10V, 101 -> +/-10.8V
//
//  // For more info see http://www.analog.com/static/imported-files/data_sheets/AD5724_5734_5754.pdf
//
//  // DAC power control register (all ch + ref powered up)
//  digitalWrite(DAC2_PIN, LOW);
//  SPI.transfer(0x10);
//  SPI.transfer(0x00);
//  SPI.transfer(0x1f);
//  digitalWrite(DAC2_PIN, HIGH);
//
//  // DAC control register (SDO turned off)
//  digitalWrite(DAC2_PIN, LOW);
//  SPI.transfer(0x19); // write to control register
//  SPI.transfer(0x00);
//  SPI.transfer(0x0d); // TSD (thermal shutdown) enable, Clamp (current limit) enable, Clear to 0V, SDO (serial data out) disabled
//  digitalWrite(DAC2_PIN, HIGH);
//
//  /* DAC output range register (all ch +/-5V range)*/
//  digitalWrite(DAC2_PIN, LOW);
//  SPI.transfer(0x0c); // all four DACs
//  // 0x08 = DAC1, 0x09 = DAC2, 0x0a = DAC3, 0x0b = DAC4, 0x0c = all DACs
//  SPI.transfer(0x00);
//  SPI.transfer(0x01);
//  // 0 = +5V range, 1 = +10V range, 2 = +10.8V range, 3 = +/-5V range
//  // 4 = +/-10V range, 5 = +/- 10.8V range
//  digitalWrite(DAC2_PIN, HIGH);
//  // set dac2 outputs to 0V
//  dac2Write(0, 0);
//  dac2Write(1, 0);
//  dac2Write(2, 0);
//  dac2Write(3, 0);


  // PC communications
  Serial.begin(115200);
  sprintf(stringBuff, "0,FR,MouseoVeR BCS firmware revision 7X4");
  printString0((char*)stringBuff);
  printNewline0();

  // LCD communication
  Serial1.begin(19200); 
  Serial1.print(0x0c, BYTE); // clear the display
  delay(100); // im ms
  Serial1.print(0x11, BYTE); // Back-light on
  Serial1.print("* Hello Xinyu *   (Ver. 8)");
  
  // VR communication
  Serial2.begin(115200);

  // Servo motor communication - multiple Zaber motors, daisy chained
  Serial3.begin(9600);

  idx = 0;
  vr_idx = 0;

  attachInterrupt(0, th1, RISING); // hardware interrupt 0 is on pin 2 - AP detection box, rising phase
  attachInterrupt(1, th2, RISING); // hardware interrupt 1 is on pin 3 - AP detection box, upper limit threshold
  attachInterrupt(2, th3, RISING); // hardware interrupt 2 is on pin 21 - AP detection box, descending phase
  //attachInterrupt(4, PM, RISING);  // hardware interrupt 4 is on pin 20 (best to keep this OFF if not connected)

  th1Timer = allocateTimer(); // timer object, for AP threshold detector
  th2Timer = allocateTimer(); // timer object, for AP threshold detector
  
  thDetect = 0;
  th1Detected = 0;
  th2Detected = 0;

  strcpy(eventstr_curr," "); // set current eventstr to a space

  // once per initialization, start the randomSeed for random();  
  // use the current usec time for the randomSeed seed (will provide a different seed each time)
  uint64_t ts;
  ts = getTimestamp(); // get the timestamp
  randomSeed(ts); // randomSeed for random();

 
  // Init done
  digitalWrite(LED_PIN, HIGH);

  delay(2000); // 2000ms delay to view LCD messages

} // END void setup()

  
/////////////////////////////////////////////////
// THE CLOCK AND DATA PROCESSING LOOP STARTS HERE 
/////////////////////////////////////////////////

void loop() {
  uint8_t c, i;

  for (;;) {

    // get the time when starting the loop
    uint64_t ts;
    ts = getTimestamp(); // get the time stamp

/*
    // Test Code for sending a test string every 10 ms to Matlab
    if ((millis() % 50) == 0) {
      if ((millis() > 10000) && (millis() < 20000) && (tflag == 0)) {
        printTimestamp();
        sprintf(stringBuff, "Time,");
        printString0((char*)stringBuff);
        printValue0_U32(millis());
        printNewline0();
        tflag = 1;
      }
    } else {
      tflag = 0;
    }
*/
/*
    // Test Code - for updating the LCD display every second with information
    new_ms = millis();
    if (new_ms - old_ms >= 1000) {
      old_ms = new_ms;
      int count = Serial.txCount();
      int s = 0;
      int m = 0;
      int h = 0;
      while (new_ms > 3600000) {
        h++;
        new_ms -= 3600000;
      }
      while (new_ms > 60000) {
        m++;
        new_ms -= 60000;
      }
      while (new_ms > 1000) {
        s++;
        new_ms -= 1000;
      }
      sprintf(stringBuff, "%c%02d:%02d:%02d  %d", 0x0c,h,m,s,count); 
      printString1((char*)stringBuff);
      // also print the count
      printTimestamp();
      sprintf(stringBuff, "txCount,");
      printString0((char*)stringBuff);
      printValue0_U32(count);
      printNewline0();
    }
*/      

    if (pm_flag) { // digital3 input　pm_flag：protocol marker
      printTime(pm_msec, pm_usec);//显示时间
      sprintf(stringBuff, "PM");//把PM存为stringBuff
      printlnString0((char*)stringBuff);
      pm_flag = 0;
    }

    if (Serial.rxStatus()) { // buffer overflow check
      sprintf(stringBuff, "PC receive buffer overflow");
      printlnString0((char*)stringBuff);
      pcrxovf++;
    }
    if (Serial2.rxStatus()) { // buffer overflow check
      sprintf(stringBuff, "VR receive buffer overflow");
      printlnString0((char*)stringBuff);
      vrrxovf++;
    }

    if (Serial.available() > 0) { // PC communication via usb (Online matlab GUI) //直到line 2315
      c = Serial.read(); // read port 
      //Serial.print(c); echo back, test code
      if (c == '\r') { // end of line //直到line 2314
        buffer[idx] = 0; // reset buffer counter
        parse((char*)buffer, argv, sizeof(argv)); // parse buffer into args,see void parse() 
        // process buffer. (read usb inputs from Matlab gui)
        if (strcmp(argv[0], "homepos") == 0) {
          device = 0;
          if (strlen(argv[1]) > 0) {
            device= atoi(argv[1]);
          }
          if ((device >= 0) && (device <= 32))
            servoHome((uint8_t)device);
        }
        else if (strcmp(argv[0], "endpos") == 0) {
          device = 0;
          if (strlen(argv[1]) > 0) {
            device= atoi(argv[1]);
          }
          if ((device >= 0) && (device <= 32))
            servoEnd((uint8_t)device);
        }
        else if (strcmp(argv[0], "moveto") == 0) {
          device= atoi(argv[1]);
          position = atol(argv[2]);
          if ((device >= 0) && (device <= 32) &&
            (position >= 0) && (position <= 533326))
            servoCommand((uint8_t)device, MOVE_ABSOLUTE_CMD, position);
        }
        else if (strcmp(argv[0], "setSpeed") == 0) {
          device= atoi(argv[1]);
          servoSpeed = atol(argv[2]);
          if ((device >= 0) && (device <= 32) &&
            (servoSpeed >= 0) && (servoSpeed <= 164000)) // safety check - 360 deg/s max
            setServoSpeed((uint8_t)device, servoSpeed);
        }
        else if (strcmp(argv[0], "setMaxPos") == 0) {
          device= atoi(argv[1]);
          maxPos = atol(argv[2]);
          if ((device >= 0) && (device <= 32) &&
            (maxPos >= 0) && (maxPos <= 533326)) // safety check 
            setMaxPos((uint8_t)device, maxPos);
        }
        else if (strcmp(argv[0], "microstepRes") == 0) {
          device= atoi(argv[1]);
          microstepRes = atol(argv[2]);
          if ((device >= 0) && (device <= 32) &&
            (microstepRes >= 0) && (microstepRes <= 126)) // safety check 
            setMicrostepRes((uint8_t)device, microstepRes);
        }
        else if (strcmp(argv[0], "speed_th") == 0) {
          speed_th_curr = atol(argv[1]);
        }
        else if (strcmp(argv[0], "speed_th_tag") == 0) {
          speed_th_tag = atoi(argv[1]);
        }
        else if (strcmp(argv[0], "lap_loop") == 0) {
          lap_loop = atoi(argv[1]);
        }
        else if (strcmp(argv[0], "lap_cnt") == 0) {
          lap_cnt = atoi(argv[1]);
        }
        
        else if (strcmp(argv[0], "lick_RD") == 0) {

          lick_RD = atoi(argv[1]);
          if (lick_RD) {          
            uart1_put(0x80); // row 0, col 0
            Serial1.print(0x0c, BYTE); // clear the display
            Serial1.print(0x80, BYTE); // move the cursor to the start         
            sprintf(stringBuff, "Lick ON!"); 
            printString1((char*)stringBuff);
          }
          else {
            uart1_put(0x80); // row 0, col 0
            Serial1.print(0x0c, BYTE); // clear the display
            Serial1.print(0x80, BYTE); // move the cursor to the start         
            sprintf(stringBuff, "Lick OFF!"); 
            printString1((char*)stringBuff);
          }         
          
        }
        
        else if (strcmp(argv[0], "lk_th") == 0) {
          lk_th = atoi(argv[1]);
        }
        else if (strcmp(argv[0], "rollMax") == 0) {
          rollMax = atoi(argv[1]);
        }
        else if (strcmp(argv[0], "randRD_flag") == 0) {
          randRD_flag = atoi(argv[1]);
        }
        else if (strcmp(argv[0], "RD_score_reset") == 0) {
          lap_cnt = 0;
          RD_cnt_curr=0;
          RD_score=0;
        }
        else if (strcmp(argv[0], "RDS_clear") == 0) {
          RDS_A1 = 0; RDS_A2 = 0; RDS_A3 = 0; RDS_A4 = 0; RDS_A14 = 0;
          RDS_B1 = 0; RDS_B2 = 0; RDS_B3 = 0; RDS_B4 = 0; RDS_B14 = 0; 
          RDS_C1 = 0; RDS_C2 = 0; RDS_C3 = 0; RDS_C4 = 0; RDS_C14 = 0;
          RDS_D1 = 0; RDS_D2 = 0; RDS_D3 = 0; RDS_D4 = 0; RDS_D14 = 0;
          RDS_A1_cnt=0; RDS_A2_cnt=0; RDS_A3_cnt=0; RDS_A4_cnt=0; RDS_A14_cnt=0;
          RDS_B1_cnt=0; RDS_B2_cnt=0; RDS_B3_cnt=0; RDS_B4_cnt=0; RDS_B14_cnt=0;
          RDS_C1_cnt=0; RDS_C2_cnt=0; RDS_C3_cnt=0; RDS_C4_cnt=0; RDS_C14_cnt=0;          
          RDS_D1_cnt=0; RDS_D2_cnt=0; RDS_D3_cnt=0; RDS_D4_cnt=0; RDS_D14_cnt=0;
          
          RDS_XZ1=0;RDS_XZ2=0;RDS_XZ11=0;RDS_XZ12=0;RDS_XZ3=0;RDS_XZ11_TR=0;RDS_XZR=0;RDS_XZTR1=0;RDS_XZTR2=0;RDS_XZTR3=0; RDS_XZRand2=0; RDS_XZRand3=0; 
          RDS_XZ1_cnt=0;RDS_XZ2_cnt=0;RDS_XZ11_cnt=0;RDS_XZ12_cnt=0;RDS_XZ3_cnt=0;RDS_XZ11_TR_cnt=0;RDS_XZR_cnt=0;RDS_XZTR1_cnt=0;RDS_XZTR2_cnt=0;RDS_XZTR3_cnt=0; RDS_XZRand2_cnt=0; RDS_XZRand3_cnt=0;
          
        }
        else if (strcmp(argv[0], "RD_cnt") == 0) {
          RD_cnt=atoi(argv[1]);
        }  
        else if (strcmp(argv[0], "RD_cnt_curr") == 0) {
          RD_cnt_curr=atoi(argv[1]);        }  
        else if (strcmp(argv[0], "RDS_A1") == 0) {
          RDS_A1_listnum = 0;
          RDS_A1_cnt=0;
          RDS_A1 = atoi(argv[1]);    
          min_RD=RDS_A1_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_A2") == 0) {
          RDS_A2_listnum = 0;
          RDS_A2_cnt=0;
          RDS_A2 = atoi(argv[1]);          
          min_RD=RDS_A2_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_A3") == 0) {
          RDS_A3_listnum = 0;
          RDS_A3_cnt=0;        
          RDS_A3 = atoi(argv[1]);          
          min_RD=RDS_A3_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_A4") == 0) {
          RDS_A4_listnum = 0;
          RDS_A4_cnt=0;        
          RDS_A4 = atoi(argv[1]);          
          min_RD=RDS_A4_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_A14") == 0) {
          RDS_A14_listnum = 0;
          RDS_A14_cnt=0;        
          RDS_A14 = atoi(argv[1]);          
          min_RD=RDS_A14_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_B1") == 0) {
          RDS_B1_listnum = 0;
          RDS_B1 = atoi(argv[1]);          
          RDS_B1_cnt=0;
          min_RD=RDS_B1_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_B2") == 0) {
          RDS_B2_listnum = 0;
          RDS_B2 = atoi(argv[1]);          
          RDS_B2_cnt=0;
          min_RD=RDS_B2_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_B3") == 0) {
          RDS_B3_listnum = 0;        
          RDS_B3 = atoi(argv[1]);          
          RDS_B3_cnt=0;
          min_RD=RDS_B3_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_B4") == 0) {
          RDS_B4_listnum = 0;        
          RDS_B4 = atoi(argv[1]);          
          RDS_B4_cnt=0;
          min_RD=RDS_B4_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_B14") == 0) {
          RDS_B14_listnum = 0;        
          RDS_B14 = atoi(argv[1]);          
          RDS_B14_cnt=0;
          min_RD=RDS_B14_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_C1") == 0) {
          RDS_C1_listnum = 0;
          RDS_C1 = atoi(argv[1]);          
          RDS_C1_cnt=0;
          min_RD=RDS_C1_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_C2") == 0) {
          RDS_C2_listnum = 0;
          RDS_C2 = atoi(argv[1]);          
          RDS_C2_cnt=0;
          min_RD=RDS_C2_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_C3") == 0) {
          RDS_C3_listnum = 0;        
          RDS_C3 = atoi(argv[1]);          
          RDS_C3_cnt=0;
          min_RD=RDS_C3_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_C4") == 0) {
          RDS_C4_listnum = 0;        
          RDS_C4 = atoi(argv[1]);          
          RDS_C4_cnt=0;
          min_RD=RDS_C4_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_C14") == 0) {
          RDS_C14_listnum = 0;        
          RDS_C14 = atoi(argv[1]);          
          RDS_C14_cnt=0;
          min_RD=RDS_C14_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_D1") == 0) {
          RDS_D1_listnum = 0;
          RDS_D1 = atoi(argv[1]);          
          RDS_D1_cnt=0;
          min_RD=RDS_D1_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_D2") == 0) {
          RDS_D2_listnum = 0;
          RDS_D2 = atoi(argv[1]);          
          RDS_D2_cnt=0;
          min_RD=RDS_D2_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_D3") == 0) {
          RDS_D3_listnum = 0;        
          RDS_D3 = atoi(argv[1]);          
          RDS_D3_cnt=0;
          min_RD=RDS_D3_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_D4") == 0) {
          RDS_D4_listnum = 0;        
          RDS_D4 = atoi(argv[1]);          
          RDS_D4_cnt=0;
          min_RD=RDS_D4_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_D14") == 0) {
          RDS_D14_listnum = 0;        
          RDS_D14 = atoi(argv[1]);          
          RDS_D14_cnt=0;
          min_RD=RDS_D14_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_E1") == 0) {
          RDS_E1_listnum = 0;
          RDS_E1 = atoi(argv[1]);          
          RDS_E1_cnt=0;
          min_RD=RDS_E1_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_E2") == 0) {
          RDS_E2_listnum = 0;
          RDS_E2 = atoi(argv[1]);          
          RDS_E2_cnt=0;
          min_RD=RDS_E2_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_E3") == 0) {
          RDS_E3_listnum = 0;        
          RDS_E3 = atoi(argv[1]);          
          RDS_E3_cnt=0;
          min_RD=RDS_E3_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_E4") == 0) {
          RDS_E4_listnum = 0;        
          RDS_E4 = atoi(argv[1]);          
          RDS_E4_cnt=0;
          min_RD=RDS_E4_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        else if (strcmp(argv[0], "RDS_E14") == 0) {
          RDS_E14_listnum = 0;        
          RDS_E14 = atoi(argv[1]);          
          RDS_E14_cnt=0;
          min_RD=RDS_E14_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        
        else if (strcmp(argv[0], "RDS_XZ1") == 0) {
          randRD_flag=0;  //no random reward
          RDS_XZ1_listnum = 0;
          RDS_XZ1_cnt=0;
          RDS_XZ1 = atoi(argv[1]);  //argv[1] is either 0 or 1, turning the RDS OFF/ON    
          min_RD=RDS_XZ1_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        
        
        else if (strcmp(argv[0], "RDS_XZ11") == 0) {
          randRD_flag=0;  //no random reward
          RDS_XZ11_listnum = 0;
          RDS_XZ11_cnt=0;
          RDS_XZ11 = atoi(argv[1]);  //argv[1] is either 0 or 1, turning the RDS OFF/ON    
          min_RD=RDS_XZ11_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        
        else if (strcmp(argv[0], "RDS_XZ12") == 0) {
          randRD_flag=0;  //no random reward
          RDS_XZ12_listnum = 0;
          RDS_XZ12_cnt=0;
          RDS_XZ12 = atoi(argv[1]);  //argv[1] is either 0 or 1, turning the RDS OFF/ON    
          min_RD=RDS_XZ12_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }

        else if (strcmp(argv[0], "RDS_XZ3") == 0) {
          randRD_flag=0;  //no random reward
          RDS_XZ3_listnum = 0;
          RDS_XZ3_cnt=0;
          RDS_XZ3 = atoi(argv[1]);  //argv[1] is either 0 or 1, turning the RDS OFF/ON    
          min_RD=RDS_XZ3_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        
        else if (strcmp(argv[0], "RDS_XZ11_TR") == 0) {
          randRD_flag=0;  //no random reward
          RDS_XZ11_TR_listnum = 0;
          RDS_XZ11_TR_cnt=0;
          RDS_XZ11_TR = atoi(argv[1]);  //argv[1] is either 0 or 1, turning the RDS OFF/ON    
          min_RD=RDS_XZ11_TR_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        
        else if (strcmp(argv[0], "RDS_XZ2") == 0) {
          randRD_flag=0;  //no random reward
          RDS_XZ2_listnum = 0;
          RDS_XZ2_cnt=0;
          RDS_XZ2 = atoi(argv[1]);  //argv[1] is either 0 or 1, turning the RDS OFF/ON    
          min_RD=RDS_XZ2_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
                
        else if (strcmp(argv[0], "RDS_XZTR1") == 0) {
          randRD_flag=0;  //no random reward
          RDS_XZTR1_listnum = 0;
          RDS_XZTR1_cnt=0;
          RDS_XZTR1 = atoi(argv[1]);  //argv[1] is either 0 or 1, turning the RDS OFF/ON    
          min_RD=RDS_XZTR1_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
                
        else if (strcmp(argv[0], "RDS_XZTR2") == 0) {
          randRD_flag=0;  //no random reward
          RDS_XZTR2_listnum = 0;
          RDS_XZTR2_cnt=0;
          RDS_XZTR2 = atoi(argv[1]);  //argv[1] is either 0 or 1, turning the RDS OFF/ON    
          min_RD=RDS_XZTR2_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }

        else if (strcmp(argv[0], "RDS_XZTR3") == 0) {
          randRD_flag=0;  //no random reward
          RDS_XZTR3_listnum = 0;
          RDS_XZTR3_cnt=0;
          RDS_XZTR3 = atoi(argv[1]);  //argv[1] is either 0 or 1, turning the RDS OFF/ON    
          min_RD=RDS_XZTR3_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
        }
        
        else if (strcmp(argv[0], "RDS_XZRand2") == 0) {
          RDS_XZRand2_listnum = 0;
          RDS_XZRand2_cnt=0;
          RDS_XZRand2 = atoi(argv[1]);  //argv[1] is either 0 or 1, turning the RDS OFF/ON    
          min_RD=RDS_XZRand2_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
          if (RDS_XZRand2){
            randRD_flag=1;  //random reward ON
          }
          else {
            randRD_flag=0;  //random reward OFF
          }
        }
        
        else if (strcmp(argv[0], "RDS_XZRand3") == 0) {
          RDS_XZRand3_listnum = 0;
          RDS_XZRand3_cnt=0;
          RDS_XZRand3 = atoi(argv[1]);  //argv[1] is either 0 or 1, turning the RDS OFF/ON    
          min_RD=RDS_XZRand3_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
          if (RDS_XZRand3){
            randRD_flag=1;  //random reward ON
          }
          else {
            randRD_flag=0;  //random reward OFF
          }
        }
        
        else if (strcmp(argv[0], "RDS_XZR") == 0) {
          RDS_XZR_listnum = 0;
          RDS_XZR_cnt=0;
          RDS_XZR = atoi(argv[1]);  //argv[1] is either 0 or 1, turning the RDS OFF/ON    
          min_RD=RDS_XZR_num-1;  // set "min_RD" per lap (substract 1 for p0 - lapmarker)
          if (RDS_XZR){
            randRD_flag=1;  //random reward ON
            th_binomial=10; //For binomial RD protocol: if this protocol is turned on, set the reward probability for each individual new location (photobeam) to be 10% 
            // num_randRD=4; //For the fixed RD number protocol: 4 RDs per lap
          }
          else {
            randRD_flag=0;  //random reward OFF
          }
        }
        
        else if (strcmp(argv[0], "ASS_A") == 0) {
          dac2Write(0,0);
          dac2Write(1,0);
          ASS_A = atoi(argv[1]);
        }
        else if (strcmp(argv[0], "stim_zone") == 0) {
          stim_zone_curr = atoi(argv[1]); // selected stim_zone location (1-3)
        }
        else if (strcmp(argv[0], "stim") == 0) {
          stimZone_flag = atoi(argv[1]); // 1 for ON, 0 for OFF 
        }
        else if (strcmp(argv[0], "stim_i") == 0) {
          stim_i = atoi(argv[1]); // intracellular inter-stim interval, in microsec
        }
        else if (strcmp(argv[0], "stim_d") == 0) {
          stim_i = atoi(argv[1]); // intracellular stim duration, in microsec
        }
        else if (strcmp(argv[0], "DAC1") == 0) { // not wired up to anything 
          dac1Write(atoi(argv[1]),atoi(argv[2]));
        }
        else if (strcmp(argv[0], "DAC2") == 0) { // control DAC2 output, 16-bit DAC (value 0-65536 = 0-10V)
          dac2Write(atoi(argv[1]),atoi(argv[2]));
        }
        else if (strcmp(argv[0], "startTraining") == 0) {
          endpos = 400000; // hardcoded for now
          homepos = 1; // hardcoded for now
          training = 1;
        }
        else if (strcmp(argv[0], "stopTraining") == 0) {
          training = 0;
          servoCommand(1,STOP_CMD,0);
        }
        else if (strcmp(argv[0], "reward") == 0) {
          printRD();
        }
        else if (strcmp(argv[0], "videoRecON") == 0) {
          videoRecON();
        }
        else if (strcmp(argv[0], "videoRecOFF") == 0) {
          videoRecOFF();
        }
        else if (strcmp(argv[0], "servoRenumber") == 0) {
          servoRenumber();
        }
        else if (strcmp(argv[0], "servoReset") == 0) {
           device = atoi(argv[1]);
          servoReset(device);
        }
        else if (strcmp(argv[0], "servoStop") == 0) {
          device = atoi(argv[1]);
          servoStop(device);
        }
        else if (strcmp(argv[0], "loopTimeMaxReset") == 0) {
          loopTimeMax = 0;
        }
        else if (strcmp(argv[0], "overflow") == 0) {
          printValue0_U32((uint32_t)pcrxovf);
          uart0_put(',');
          printValue0_U32((uint32_t)vrrxovf);
          printNewline0();
        }
        else if (strcmp(argv[0], "maxtime") == 0) {
          printValue0_U32(loopTimeMax);
          printNewline0();
        }
        else if (strcmp(argv[0], "maxbuffer") == 0) {
          printValue0_U32((uint32_t)Serial.rxMax());
          uart0_put(',');
          printValue0_U32((uint32_t)Serial1.rxMax());
          uart0_put(',');
          printValue0_U32((uint32_t)Serial2.rxMax());
          uart0_put(',');
          printValue0_U32((uint32_t)Serial3.rxMax());
          printNewline0();
        }
        else if (strcmp(argv[0], "timestamp") == 0) {
          uint64_t ts;
          // get the time stamp
          ts = getTimestamp();
          Serial.println((uint32_t)(ts/1000));
          Serial.println((uint16_t)(ts % 1000));
          uint32_t msec = millis();
          Serial.println(msec);
        }
        idx = 0;
      }
      else if (((c == '\b') || (c == 0x7f)) && (idx > 0)) {
        idx--;
      } 
      else if ((c >= ' ') && (idx < sizeof(buffer) - 1)) {
        buffer[idx++] = c;
      }
    } // END serial loop for PC communication
    
    if (Serial2.available() > 0) { // VR communication (MouseoVeR)
      c = Serial2.read();
      if ((c == '\n') || (c == '\r')) {
        printTimestamp(); //write time stamp to the PC
        // process buffer
        vrbuffer[vr_idx] = lastVrbuffer[vr_idx] = 0;//从ＶＲ传过来的字符串
        parse((char*)vrbuffer, argv, sizeof(argv));//把从ＶＲ传过来的字符串拆分开来
        if ((strcmp(argv[1], "Display_Blanking_On") == 0) ||   //not currently used
          (strcmp(argv[1], "Display_Blanking_Off") == 0) ||
          (strcmp(argv[1], "Enable_Motion_Off") == 0) ||
          (strcmp(argv[1], "Enable_Motion_On") == 0)) {
          printlnString0(argv[1]);
        } 
        else {
          //send vrbuffer data
          sprintf(stringBuff, "VR,"); // indicate VR string, data is streaming from MouseoVeR at the frame rate of the projected images.
          printString0((char*)stringBuff);  //write a string "VR" to indicate that the following information is from VR
          for (i=0; i<vr_idx; i++) {
            uart0_put(lastVrbuffer[i]);  //write the VR string to the PC
            // see below vr_time, vr_x, vr_y etc. for the definition of each element in the VR string array. The total number of strings could be more than 7 dependeing on how many VR events occured. 
          }
          printNewline0();  //the end of the string (time stamp+VR string) to the PC. The end of a line ('\r') is recognized by the MATLAB code (terminator='\r')

          vr_time = getValue(argv[0]);
          vr_x = getValue(argv[1]);
          vr_y = getValue(argv[2]);
          vr_z = getValue(argv[3]);
          vr_speed = getValue(argv[4]);
          vr_direction = getValue(argv[5]);
          vr_eventcnt = getValue(argv[7]);
          
          speed_buffer[speed_buffer_index] = vr_speed;
          if (speed_buffer_index < 19) {
            speed_buffer_index++;
          }
          else {
            speed_buffer_index = 0;
          }
 
          total_speed = 0;
          for (int i=0; i<20; i++) {
            total_speed = total_speed + speed_buffer[i];
          }
          avg_speed = total_speed/20;


          if ((vr_eventcnt == 1) || (vr_eventcnt == 2) || (vr_eventcnt == 3) || (vr_eventcnt == 4)) {  //Jeremy's code, not sure why read in this way... at most 2 VR events for any given time?
            strcpy(eventstr_1, argv[8]);   // the first event string is the virtual photobeam breaker　把ＶＲ字符串拆分的第８项赋值给eventstr_1
          }
          if ((vr_eventcnt == 2) || (vr_eventcnt == 3) || (vr_eventcnt == 4)) {
            strcpy(eventstr_2, argv[9]);
          }
//          if ((vr_eventcnt == 3) || (vr_eventcnt == 4)) {
//            strcpy(eventstr_3, argv[9]);
//          }
//          if (vr_eventcnt == 4) {
//            strcpy(eventstr_4, argv[10]);
//          }
          
          
          // check for new event crossed in VR
          if (vr_eventcnt>=1) {
            
            if (strcmp(eventstr_1,eventstr_curr) != 0)  {    // there is an event and it's different than the last one. 
              strcpy(eventstr_curr, eventstr_1); // if so, update current string
              
              uart1_put(0x80); // row 0, col 0
              Serial1.print(0x0c, BYTE); // clear the display　这个ｓｅｒｉａｌ１是ＬＣＤ显示屏
              Serial1.print(0x80, BYTE); // move the cursor to the start         
              sprintf(stringBuff, eventstr_1); // 把eventstr_1写入stringBuff
              printString1((char*)stringBuff); // display eventstr_curr (location of the animal in terms of photobeams/ p0, p1, p2, ...)                          
            
              /*
              Serial1.print(0x94, BYTE); //move the cursur to the second line.   //used to debug some problems of lap counting
              if (lap_tag==1) {
                sprintf(stringBuff, "1");
              }
              else {
                sprintf(stringBuff, "0");
              }
              printString1((char*)stringBuff); 
              */ 
              
              if (RD_zone_tag) {                
                // RD_zone_tag==1 means the animal was at a reward zone. Now it's a different event, which means the animal just moved out of the reward zone.
                RD_zone_tag = 0;  //reset the tag 
               
                if ((RDS_XZ2)&&(!RD_tag)) {  
                  //!RD_tag because if the reward_listnum was already moved by reward delivery, it should not be moved again by the animal moving out of the reward zone.
                  if (RDS_XZ2_listnum < (RDS_XZ2_num -1)) { 
                    //This is the second criterion: the animal moved out of the reward zone.
                    //The first criterion to move to the next location: Reward has been delivered. See the lick-dependent (lick_RD==1) RDS_XZ2 definition.                  
                    RDS_XZ2_listnum++;   //move to the next reward location
                  }
                  else {
                    RDS_XZ2_listnum=0;  //if it's already the last reward location, reset to the first
                  }
                }
                
                
                if ((RDS_XZ12)&&(!RD_tag)) {  
                  //!RD_tag because if the reward_listnum was already moved by reward delivery, it should not be moved again by the animal moving out of the reward zone.
                  if (RDS_XZ12_listnum < (RDS_XZ12_num -1)) { 
                    //This is the second criterion: the animal moved out of the reward zone.
                    //The first criterion to move to the next location: Reward has been delivered. See the lick-dependent (lick_RD==1) RDS_XZ2 definition.                  
                    RDS_XZ12_listnum++;   //move to the next reward location
                  }
                  else {
                    RDS_XZ12_listnum=0;  //if it's already the last reward location, reset to the first
                  }
                }
               
              }              
              
              RD_tag= 0 ; //reset reward tag. So another reward was allowed to be delivered when the animal moves to a new location.
              lk_cnt= 0 ;  //restart counting licks when the animal moves to a new location.                
                              
              if (RDS_XZ2) {
                Serial1.print(0xa0, BYTE); //move the cursur to the second line, position 12.
                sprintf(stringBuff, "XZ2");
                printString1((char*)stringBuff);    
                Serial1.print(0x9c, BYTE); //move the cursur to the second line, position 8.
                if (RDS_XZ2_listnum==0) {
                  sprintf(stringBuff, "0");
                }
                if (RDS_XZ2_listnum==1) {
                  sprintf(stringBuff, "1");
                }
                if ((RDS_XZ2_listnum<0)||(RDS_XZ2_listnum>1)) {
                  sprintf(stringBuff, "ER");          //to show the error
                }
                printString1((char*)stringBuff);                    
              }                

              if (lick_RD) {                
                Serial1.print(0x94, BYTE); //move the cursur to the second line.
                sprintf(stringBuff, "LK");
                printString1((char*)stringBuff);
                
                Serial1.print(0x97, BYTE); //move the cursur to position 3 (counted from 0), the second line.
                if (lk_th==5) {
                  sprintf(stringBuff, "5"); //show the lick threshold
                }                
                else if (lk_th==4) {
                  sprintf(stringBuff, "4"); //show the lick threshold
                }                
                else if (lk_th==3) {
                  sprintf(stringBuff, "3"); //show the lick threshold
                }
                else if (lk_th==2) {
                  sprintf(stringBuff, "2"); //show the lick threshold
                }              
                else if (lk_th==1) {
                  sprintf(stringBuff, "1"); //show the lick threshold
                }                
                else if (lk_th>5) {
                  sprintf(stringBuff, "M"); //show the lick threshold
                }
                printString1((char*)stringBuff);
                
                Serial1.print(0x99, BYTE); //move the cursur to position 5 (counted from 0), the second line.
                if (lk_cnt==0) {
                  sprintf(stringBuff, "0"); //show the lick count;
                }
                else if (lk_cnt==1){
                  sprintf(stringBuff, "1");  
                }
                else if (lk_cnt==2) {
                  sprintf(stringBuff, "2");
                }
                else if (lk_cnt==3) {
                  sprintf(stringBuff, "3");
                }
                else if (lk_cnt>3) {
                  sprintf(stringBuff, "M");
                }                                      
                printString1((char*)stringBuff);                
              }           
                        
              // set the random RD roll tag
              randRD_roll_tag = 1;          //roll the dice to decide whether to give a random reward ONLY when the animal entered a new location. 
              new_event_tag = 1;  //the animal comes to a new location
              
              // check for stim_zone flag, On=1, Off=0
              if (stimZone_flag) {
                
                //uart1_put(0x80); // row 0, col 0
                //Serial1.print(0x0c, BYTE); // clear the display
                //sprintf(stringBuff, "Call StimZone"); // 
                //printString1((char*)stringBuff);
                
                stimZone();  // checks if inside stimZone 
              }
            }            
            else {
              randRD_roll_tag=0;  //if the animal stays in the same position, set the tag to 0. So only one random decision is performed for every location.               
              new_event_tag = 0; //the animal stays at the same location              
            }
    
          }
          else {                //if there is no event (animal was between two consecutive photobeams), consider it as "stay at the same location"
            randRD_roll_tag=0;
            new_event_tag = 0;
          }
          
//        check for lap marker
//        old algorithm for lap counting: count 1 every time the animal encountered p0 after it encountered p20 (see Jeremy's code below)
//        to deal with the case when some photobeams may be missed, I switch to a new algorithm:
//        Count 1 when the animal encountered p0 or p1 after it encountered p16 or p17. I have 32 photobeams in the maze. p16/17 is the middle.
//        Keep in mind that is an animal tuen 180 degree back at some point, it may or may not be counted as a "lap", depending on if the animal turns before it reaches the middle point 

          if (!RDS_XZ2) {
            if (  (lap_tag) && (  (strcmp(eventstr_1,"p0") == 0) || (strcmp(eventstr_1,"p1") == 0)  )  ) { 
              lapMarker(); // updates lap_cnt and lap_tag
              //see the lapMarker(). It will add 1 to lap_cnt, set lap_tag to 0 and write this update information to the PC
            }
            
//          reset lap marker
            if ((strcmp(eventstr_1,"p16") == 0) || (strcmp(eventstr_2,"p16") == 0)||(strcmp(eventstr_1,"p17") == 0) || (strcmp(eventstr_2,"p17") == 0)) {  //sometimes could be eventstr_2? if so, why not the same thing for the lapmarker detection?
              lap_tag=1; // reset lap tag 
              lk_tag=0; // reset lick tag
            }          
          }
          
          if (RDS_XZ2) {    //the symmetric maze has fewer photobeams
              if (  (lap_tag) && (  (strcmp(eventstr_1,"p0") == 0) || (strcmp(eventstr_1,"p1") == 0)  )  ) { 
                lapMarker(); // updates lap_cnt and lap_tag
                //see the lapMarker(). It will add 1 to lap_cnt, set lap_tag to 0 and write this update information to the PC
              }
          
              //        reset lap marker
              if ((strcmp(eventstr_1,"p13") == 0) || (strcmp(eventstr_2,"p13") == 0)||(strcmp(eventstr_1,"p14") == 0) || (strcmp(eventstr_2,"p14") == 0)) {  //sometimes could be eventstr_2? if so, why not the same thing for the lapmarker detection?
                lap_tag=1; // reset lap tag 
                lk_tag=0; // reset lick tag
              }             
          }
          
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Jeremy's code          
//          if ((lap_tag) && (strcmp(eventstr_1,"p0") == 0)) { 
//            lapMarker(); // updates lap_cnt and lap_tag
//          }
//          
////        reset lap marker
//          if ((strcmp(eventstr_1,"p20") == 0) || (strcmp(eventstr_2,"p20") == 0)) {
//            lap_tag=1; // reset lap tag
//            lk_tag=0; // reset lick tag
//          }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////
// Process behavioral contingency schedules below
/////////////////////////////////////////////////
       
        ///////////////////////////////////////////////////////// 
        if (lick_RD == 0) { // NON LICK-DEPENDENT REWARD SCHEUDLE
//        1. State codes for rat position-contingent reward schedule in oval-track and linear-track (RDS_A)
          if (RDS_A1) {
            if ((strcmp(eventstr_1,&RDS_A1_list[RDS_A1_listnum][0]) == 0) ||
              (strcmp(eventstr_2,&RDS_A1_list[RDS_A1_listnum][0]) == 0)) {
              if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
              }
              else {
                printRD();
              }
              if (RDS_A1_listnum < (RDS_A1_num -1)) {
                RDS_A1_listnum++;
              }
              else {
                RDS_A1_listnum=0;
              }
            }
          }
         
          if (RDS_A2) {
            if (speed_th_tag) {
              speed_th_curr = 5000;
            }
            if (RD_cnt < 150) {
              if ((strcmp(eventstr_1,&RDS_A2_list[RDS_A2_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_A2_list[RDS_A2_listnum][0]) == 0)) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();
                }
                if (RDS_A2_listnum < (RDS_A2_num -1)) {
                  RDS_A2_listnum++;
                }
                else {
                  RDS_A2_listnum=0;
                }
              }
            }
            else {
              RDS_A2 = 0;
              RDS_A3 = 1;
            }
          } 
          
          if (RDS_A3) {
            if (RD_cnt < 180) {
              if (speed_th_tag) {
              speed_th_curr = 4800;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 210) {
              if (speed_th_tag) {
                speed_th_curr = 4600;
              }                
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 240) {
              if (speed_th_tag) {
                speed_th_curr = 4400;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if ((strcmp(eventstr_1,"p6") ==0) || (strcmp(eventstr_1,"p31") == 0) || // in RDS_A4, p7 and p15 locations trigger RD regardless of avg_speed
                    (strcmp(eventstr_2,"p6") ==0) || (strcmp(eventstr_2,"p31") == 0)) { 
                  printRD();
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 270) {
              if (speed_th_tag) {
                speed_th_curr = 4200;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 300) {
              if (speed_th_tag) {
                speed_th_curr = 4000;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 330) {
              if (speed_th_tag) {
                speed_th_curr = 3900;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 360) {
              if (speed_th_tag) {
                speed_th_curr = 3800;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 390) {
              if (speed_th_tag) {
                speed_th_curr = 3700;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 420) {
              if (speed_th_tag) {
                speed_th_curr = 3600;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 450) {
              if (speed_th_tag) {
                speed_th_curr = 3500;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 480) {
              if (speed_th_tag) {
                speed_th_curr = 3400;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 500) {
              if (speed_th_tag) {
                speed_th_curr = 3300;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else {
              RDS_A3 = 0;
              RDS_A4 = 1;
              training = 0;
              uart1_put(0x80); // row 0, col 0
              Serial1.print(0x0c, BYTE); // clear the display
              sprintf(stringBuff, "* RDS_A4 *"); // 
              printString1((char*)stringBuff);
            }
          }
           
          if (RDS_A4) {
            if (RD_cnt < 540) {
              if (speed_th_tag) {
              speed_th_curr = 3100;
              }
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
              
              if (randRD_flag) { // flag for randRD_roll script to deliver random rewards.
                randRD_roll();
              }
            }
            else if (RD_cnt < 570) {
              if (speed_th_tag) {
                speed_th_curr = 3000;
              }
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                    (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                    (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                    (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 600) {
              if (speed_th_tag) {
                speed_th_curr = 2900; 
              }
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 630) {
              if (speed_th_tag) {
                speed_th_curr = 2800; 
              }
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 660) {
              if (speed_th_tag) {
                speed_th_curr = 2800; 
              }  
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 690) {
              if (speed_th_tag) {
                speed_th_curr = 2700;
              }
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 720) {
              if (speed_th_tag) {
                speed_th_curr = 2600;
              }
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
            }
            else {
              if (speed_th_tag) {
                speed_th_curr = 2500;
              }
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
            }
          }
              
          if (RDS_A14) {
            if ((strcmp(eventstr_1,&RDS_A14_list[RDS_A14_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_A14_list[RDS_A14_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_A14_list[RDS_A14_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_A14_list[RDS_A14_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
              if (RDS_A14_listnum < (RDS_A14_num -1)) {
                RDS_A14_listnum++;
              }
              else {
                RDS_A14_listnum=0;
              }
            }
          }
             
          
//        2. State codes for rat position-contingent reward schedule in L-track (RDS_B)
          if (RDS_B1) {
            if ((strcmp(eventstr_1,&RDS_B1_list[RDS_B1_listnum][0]) == 0) ||
              (strcmp(eventstr_2,&RDS_B1_list[RDS_B1_listnum][0]) == 0)) {
              if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
              }
              else {
                printRD();
              }
              if (RDS_B1_listnum < (RDS_B1_num -1)) {
                RDS_B1_listnum++;
              }
              else {
                RDS_B1_listnum=0;
              }
            }
          }
         
          if (RDS_B2) {
            if (speed_th_tag) {
              speed_th_curr = 5000;
            }
            if (RD_cnt < 150) {
              if ((strcmp(eventstr_1,&RDS_B2_list[RDS_B2_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_B2_list[RDS_B2_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_B2_list[RDS_B2_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_B2_list[RDS_B2_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B2_listnum < (RDS_B2_num -1)) {
                  RDS_B2_listnum++;
                }
                else {
                  RDS_B2_listnum=0;
                }
              }
            }
            else {
             RDS_B2 = 0;
             RDS_B3 = 1;            
            }
          }
                 
          if (RDS_B3) {
            if (RD_cnt < 180) {
              if (speed_th_tag) {
                speed_th_curr = 4800;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 210) {
              if (speed_th_tag) {
                speed_th_curr = 4600;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 240) {
              if (speed_th_tag) {
                speed_th_curr = 4400;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 270) {
              if (speed_th_tag) {
                speed_th_curr = 4200;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 300) {
              if (speed_th_tag) {
                speed_th_curr = 4000;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 330) {
              if (speed_th_tag) {
                speed_th_curr = 3900;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 360) {
              if (speed_th_tag) {
                speed_th_curr = 3800;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 390) {
              if (speed_th_tag) {
                speed_th_curr = 3700;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 420) {
              if (speed_th_tag) {
                speed_th_curr = 3600;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 450) {
              if (speed_th_tag) {
                speed_th_curr = 3500;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 480) {
              if (speed_th_tag) {
                speed_th_curr = 3400;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 500) {
              if (speed_th_tag) {
                speed_th_curr = 3300;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else {
              RDS_B3 = 0;
              RDS_B4 = 1;
              training = 0;
              uart1_put(0x80); // row 0, col 0
              Serial1.print(0x0c, BYTE); // clear the display
              sprintf(stringBuff, "* RDS_B4 *"); // 
              printString1((char*)stringBuff);
            }
          }
           
          if (RDS_B4) {
            if (RD_cnt < 540) {
              if (speed_th_tag) {
                speed_th_curr = 3200;
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
              
              if (randRD_flag) {
                randRD_roll();
              }
            }
            
            else if (RD_cnt < 570) {
              if (speed_th_tag) {
                speed_th_curr = 3100;
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                    (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                    (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                    (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 600) {
              if (speed_th_tag) {
                speed_th_curr = 3000; 
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 630) {
              if (speed_th_tag) {
                speed_th_curr = 2900; 
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 660) {
              if (speed_th_tag) {
                speed_th_curr = 2800; 
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 690) {
              if (speed_th_tag) {
                speed_th_curr = 2700;
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 720) {
              if (speed_th_tag) {
                speed_th_curr = 2600;
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
            }
            else {
              if (speed_th_tag) {
                speed_th_curr = 2500;
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
            }
          }
          
          if (RDS_B14) {
            if ((strcmp(eventstr_1,&RDS_B14_list[RDS_B14_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_B14_list[RDS_B14_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_B14_list[RDS_B14_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_B14_list[RDS_B14_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
              if (RDS_B14_listnum < (RDS_B14_num -1)) {
                RDS_B14_listnum++;
              }
              else {
                RDS_B14_listnum=0;
              }
            }
          }


//        3. State codes for rat position-contingent reward schedule in fig8-track (RDS_C)
          if (RDS_C1) {
            if ((strcmp(eventstr_1,&RDS_C1_list[RDS_C1_listnum][0]) == 0) ||
              (strcmp(eventstr_2,&RDS_C1_list[RDS_C1_listnum][0]) == 0)) {
              if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
              }
              else {
                printRD();
              }
              if (RDS_C1_listnum < (RDS_C1_num -1)) {
                RDS_C1_listnum++;
              }
              else {
                RDS_C1_listnum=0;
              }
            }
          }
  
          if (RDS_C2) {
            if (speed_th_tag) {
              speed_th_curr = 5000;
            }
            if (RD_cnt < 150) {
              if ((strcmp(eventstr_1,&RDS_C2_list[RDS_C2_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_C2_list[RDS_C2_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_C2_list[RDS_C2_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_C2_list[RDS_C2_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C2_listnum < (RDS_C2_num -1)) {
                  RDS_C2_listnum++;
                }
                else {
                  RDS_C2_listnum=0;
                }
              }
            }
            else {
              RDS_C2 = 0;
              RDS_C3 = 1;
            }
          }           
                   
          if (RDS_C3) {
            if (RD_cnt < 180) {
              if (speed_th_tag) {
                speed_th_curr = 4800;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 210) {
              if (speed_th_tag) {
                speed_th_curr = 4600;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 240) {
              if (speed_th_tag) {
                speed_th_curr = 4400;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 270) {
              if (speed_th_tag) {
                speed_th_curr = 4200;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 300) {
              if (speed_th_tag) {
                speed_th_curr = 4000;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 330) {
              if (speed_th_tag) {
                speed_th_curr = 3900;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 360) {
              if (speed_th_tag) {
                speed_th_curr = 3800;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 390) {
              if (speed_th_tag) {
                speed_th_curr = 3700;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 420) {
              if (speed_th_tag) {
                speed_th_curr = 3600;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 450) {
              if (speed_th_tag) {
                speed_th_curr = 3500;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 480) {
              if (speed_th_tag) {
                speed_th_curr = 3400;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 500) {
              if (speed_th_tag) {
                speed_th_curr = 3300;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else {
              RDS_C3 = 0;
              RDS_C4 = 1;
              training = 0;
              uart1_put(0x80); // row 0, col 0
              Serial1.print(0x0c, BYTE); // clear the display
              sprintf(stringBuff, "* RDS_C4 *"); // 
              printString1((char*)stringBuff);
            }
          }
           
          if (RDS_C4) {
            if (RD_cnt < 540) {
              if (speed_th_tag) {
                speed_th_curr = 3200;
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
              if (randRD_flag) {
                randRD_roll();
              }
            }
            else if (RD_cnt < 570) {
              if (speed_th_tag) {
                speed_th_curr = 3100;
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                    (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                    (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                    (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 600) {
              if (speed_th_tag) {
                speed_th_curr = 3000; 
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 630) {
              if (speed_th_tag) {
                speed_th_curr = 2900; 
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 660) {
              if (speed_th_tag) {
                speed_th_curr = 2800; 
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 690) {
              if (speed_th_tag) {
                speed_th_curr = 2700;
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 720) {
              if (speed_th_tag) {
                speed_th_curr = 2600;
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
            }
            else {
              if (speed_th_tag) {
                speed_th_curr = 2500;
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
            }
          }
          
          if (RDS_C14) {
            if ((strcmp(eventstr_1,&RDS_C14_list[RDS_C14_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_C14_list[RDS_C14_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_C14_list[RDS_C14_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_C14_list[RDS_C14_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
              if (RDS_C14_listnum < (RDS_C14_num -1)) {
                RDS_C14_listnum++;
              }
              else {
                RDS_C14_listnum=0;
              }
            }
          }


//        4. State codes for rat position-contingent reward schedule in oval-track (RDS_D)
          if (RDS_D1) {
            if ((strcmp(eventstr_1,&RDS_D1_list[RDS_D1_listnum][0]) == 0) ||
              (strcmp(eventstr_2,&RDS_D1_list[RDS_D1_listnum][0]) == 0) ||
              (strcmp(eventstr_3,&RDS_D1_list[RDS_D1_listnum][0]) == 0) || 
              (strcmp(eventstr_4,&RDS_D1_list[RDS_D1_listnum][0]) == 0)) {
              if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
              }
              else {
                printRD();
              }
              if (RDS_D1_listnum < (RDS_D1_num -1)) {
                RDS_D1_listnum++;
              }
              else {
                RDS_D1_listnum=0;
              }
            }
          }
         
          if (RDS_D2) {
            if (speed_th_tag) {
              speed_th_curr = 5000;
            }
            if (RD_cnt < 150) {
              if ((strcmp(eventstr_1,&RDS_D2_list[RDS_D2_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_D2_list[RDS_D2_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_D2_list[RDS_D2_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_D2_list[RDS_D2_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D2_listnum < (RDS_D2_num -1)) {
                  RDS_D2_listnum++;
                }
                else {
                  RDS_D2_listnum=0;
                }
              }
            }
            else {
              RDS_D2 = 0;
              RDS_D3 = 1;
            }
          }           
                   
          if (RDS_D3) {
            if (RD_cnt < 180) {
              if (speed_th_tag) {
                speed_th_curr = 4800;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 210) {
              if (speed_th_tag) {
                speed_th_curr = 4600;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 240) {
              if (speed_th_tag) {
                speed_th_curr = 4400;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 270) {
              if (speed_th_tag) {
                speed_th_curr = 4200;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 300) {
              if (speed_th_tag) {
                speed_th_curr = 4000;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 330) {
              if (speed_th_tag) {
                speed_th_curr = 3900;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 360) {
              if (speed_th_tag) {
                speed_th_curr = 3800;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 390) {
              if (speed_th_tag) {
                speed_th_curr = 3700;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 420) {
              if (speed_th_tag) {
                speed_th_curr = 3600;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 450) {
              if (speed_th_tag) {
                speed_th_curr = 3500;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 480) {
              if (speed_th_tag) {
                speed_th_curr = 3400;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 500) {
              if (speed_th_tag) {
                speed_th_curr = 3300;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else {
              RDS_D3 = 0;
              RDS_D4 = 1;
              training = 0;
              uart1_put(0x80); // row 0, col 0
              Serial1.print(0x0c, BYTE); // clear the display
              sprintf(stringBuff, "* RDS_D4 *"); // 
              printString1((char*)stringBuff);
            }
          }
           
          if (RDS_D4) {
            if (RD_cnt < 540) {
              if (speed_th_tag) {
                speed_th_curr = 3200;
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
              
              if (randRD_flag) {
                randRD_roll();
              }
            }
            else if (RD_cnt < 570) {
              if (speed_th_tag) {
                speed_th_curr = 3100;
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                    (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                    (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                    (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }  
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 600) {
              if (speed_th_tag) {
                speed_th_curr = 3000; 
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 630) {
              if (speed_th_tag) {
                speed_th_curr = 2900; 
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 660) {
              if (speed_th_tag) {
                speed_th_curr = 2800; 
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 690) {
              if (speed_th_tag) {
                speed_th_curr = 2700;
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 720) {
              if (speed_th_tag) {
                speed_th_curr = 2600;
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
            }
            else {
              if (speed_th_tag) {
                speed_th_curr = 2500;
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else {
                    printRD();
                  }
                }
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
            }
          }
          
          if (RDS_D14) {
            if ((strcmp(eventstr_1,&RDS_D14_list[RDS_D14_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_D14_list[RDS_D14_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_D14_list[RDS_D14_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_D14_list[RDS_D14_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();
                }
              }
              if (RDS_D14_listnum < (RDS_D14_num -1)) {
                RDS_D14_listnum++;
              }
              else {
                RDS_D14_listnum=0;
              }
            }
          }
        }
        
        
//      5. State codes for rat position-contingent reward schedule in teardrop-track (RDS_E)
        if (RDS_E4) {
          if (RD_cnt < 540) {
            if (speed_th_tag) {
              speed_th_curr = 3200;
            }
            if ((strcmp(eventstr_1,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_E4_list[RDS_E4_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_E4_list[RDS_E4_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();
                }
              }
              if (RDS_E4_listnum < (RDS_E4_num -1)) {
                RDS_E4_listnum++;
              }
              else {
                RDS_E4_listnum=0;
              }
            }
            
            if (randRD_flag) {
              randRD_roll();
            }
          }
          else if (RD_cnt < 570) {
            if (speed_th_tag) {
              speed_th_curr = 3100;
            }
            if ((strcmp(eventstr_1,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_E4_list[RDS_E4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_E4_list[RDS_E4_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();
                }
              }  
              if (RDS_E4_listnum < (RDS_E4_num -1)) {
                RDS_E4_listnum++;
              }
              else {
                RDS_E4_listnum=0;
              }
            }
          }
          else if (RD_cnt < 600) {
            if (speed_th_tag) {
              speed_th_curr = 3000; 
            }
            if ((strcmp(eventstr_1,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_E4_list[RDS_E4_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_E4_list[RDS_E4_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();
                }
              }
              if (RDS_E4_listnum < (RDS_E4_num -1)) {
                RDS_E4_listnum++;
              }
              else {
                RDS_E4_listnum=0;
              }
            }
          }
          else if (RD_cnt < 630) {
            if (speed_th_tag) {
              speed_th_curr = 2900; 
            }
            if ((strcmp(eventstr_1,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_E4_list[RDS_E4_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_E4_list[RDS_E4_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();
                }
              }
              if (RDS_E4_listnum < (RDS_E4_num -1)) {
                RDS_E4_listnum++;
              }
              else {
                RDS_E4_listnum=0;
              }
            }
          }
          else if (RD_cnt < 660) {
            if (speed_th_tag) {
              speed_th_curr = 2800; 
            }
            if ((strcmp(eventstr_1,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_E4_list[RDS_E4_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_E4_list[RDS_E4_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();
                }
              }
              if (RDS_E4_listnum < (RDS_E4_num -1)) {
                RDS_E4_listnum++;
              }
              else {
                RDS_E4_listnum=0;
              }
            }
          }
          else if (RD_cnt < 690) {
            if (speed_th_tag) {
              speed_th_curr = 2700;
            }
            if ((strcmp(eventstr_1,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_E4_list[RDS_E4_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_E4_list[RDS_E4_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();
                }
              }
              if (RDS_E4_listnum < (RDS_E4_num -1)) {
                RDS_E4_listnum++;
              }
              else {
                RDS_E4_listnum=0;
              }
            }
          }
          else if (RD_cnt < 720) {
            if (speed_th_tag) {
              speed_th_curr = 2600;
            }
            if ((strcmp(eventstr_1,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_E4_list[RDS_E4_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_E4_list[RDS_E4_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();
                }
              }
              if (RDS_E4_listnum < (RDS_E4_num -1)) {
                RDS_E4_listnum++;
              }
              else {
                RDS_E4_listnum=0;
              }
            }
          }
          else {
            if (speed_th_tag) {
              speed_th_curr = 2500;
            }
            if ((strcmp(eventstr_1,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_E4_list[RDS_E4_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_E4_list[RDS_E4_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_E4_list[RDS_E4_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();
                }
              }
              if (RDS_E4_listnum < (RDS_E4_num -1)) {
                RDS_E4_listnum++;
              }
              else {
                RDS_E4_listnum=0;
              }
            }
          }
        }
          
        if (RDS_E14) {
          if ((strcmp(eventstr_1,&RDS_E14_list[RDS_E14_listnum][0]) == 0) ||
              (strcmp(eventstr_2,&RDS_E14_list[RDS_E14_listnum][0]) == 0) ||
              (strcmp(eventstr_3,&RDS_E14_list[RDS_E14_listnum][0]) == 0) || 
              (strcmp(eventstr_4,&RDS_E14_list[RDS_E14_listnum][0]) == 0)) {
            if (avg_speed < speed_th) {
              if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
              }
              else {
                printRD();
              }
            }
            if (RDS_E14_listnum < (RDS_E14_num -1)) {
              RDS_E14_listnum++;
            }
            else {
              RDS_E14_listnum=0;
            }
          }
        }
        
        ///////////////////////////////////////////////////////////////////////////////////////////////
        // Xinyu's RDS (lick-independent)
        if (lick_RD==0) {
          
          if (RDS_XZ1) {      
              randRD_flag=0;  //do not use random reward
              if ((strcmp(eventstr_1,&RDS_XZ1_list[RDS_XZ1_listnum][0]) == 0) ||           //if any event string is the position that is what supposed to be the current reward position (defined in the RDS_XZ1_list)
                (strcmp(eventstr_2,&RDS_XZ1_list[RDS_XZ1_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_XZ1_list[RDS_XZ1_listnum][0]) == 0) ||              
                (strcmp(eventstr_4,&RDS_XZ1_list[RDS_XZ1_listnum][0]) == 0)) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();  //give reward if not "p0" (from Jeremy's code, why?)
                }
                if (RDS_XZ1_listnum < (RDS_XZ1_num -1)) {
                  RDS_XZ1_listnum++;   //move to the next reward location
                }
                else {
                  RDS_XZ1_listnum=0;  //if it's already the last reward location, reset to the first
                }
              }
            }
          
          if (RDS_XZ11) {      
              randRD_flag=0;  //do not use random reward
              if ((strcmp(eventstr_1,&RDS_XZ11_list[RDS_XZ11_listnum][0]) == 0) ||           //if any event string is the position that is what supposed to be the current reward position (defined in the RDS_XZ1_list)
                (strcmp(eventstr_2,&RDS_XZ11_list[RDS_XZ11_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_XZ11_list[RDS_XZ11_listnum][0]) == 0) ||              
                (strcmp(eventstr_4,&RDS_XZ11_list[RDS_XZ11_listnum][0]) == 0)) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();  //give reward if not "p0" (from Jeremy's code, why?)
                }
                if (RDS_XZ11_listnum < (RDS_XZ11_num -1)) {
                  RDS_XZ11_listnum++;   //move to the next reward location
                }
                else {
                  RDS_XZ11_listnum=0;  //if it's already the last reward location, reset to the first
                }
              }
            }

          if (RDS_XZ12) {      
              randRD_flag=0;  //do not use random reward
              if ((strcmp(eventstr_1,&RDS_XZ12_list[RDS_XZ2_listnum][0]) == 0) ||           //if any event string is the position that is what supposed to be the current reward position (defined in the RDS_XZ1_list)
                (strcmp(eventstr_2,&RDS_XZ12_list[RDS_XZ2_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_XZ12_list[RDS_XZ2_listnum][0]) == 0) ||              
                (strcmp(eventstr_4,&RDS_XZ12_list[RDS_XZ2_listnum][0]) == 0)) {
                //if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                //}
                //else {
                //  if ((new_event_tag == 1)&&(RDS_XZ12_listnum == 0)) {   //this RDS only has one RD location. So need to add this criterion (animal enters a new location); otherwise water will be constantly draining out
                  //Some bugs occured when just use the "new location" criterion. So I now make two RD zones, but the second one is FAKE. "RDS_XZ12_listnum==0" means only give reward at the first location.
                  if (RDS_XZ12_listnum == 0) { 
                    printRD();  
                  }
                //}
                if (RDS_XZ12_listnum < (RDS_XZ12_num -1)) {
                  RDS_XZ12_listnum++;   //move to the next reward location
                }
                else {
                  RDS_XZ12_listnum=0;  //if it's already the last reward location, reset to the first
                }
              }
            }            

          if (RDS_XZ3) {      
              randRD_flag=0;  //do not use random reward
              if ((strcmp(eventstr_1,&RDS_XZ3_list[RDS_XZ3_listnum][0]) == 0) ||           //if any event string is the position that is what supposed to be the current reward position (defined in the RDS_XZ1_list)
                (strcmp(eventstr_2,&RDS_XZ3_list[RDS_XZ3_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_XZ3_list[RDS_XZ3_listnum][0]) == 0) ||              
                (strcmp(eventstr_4,&RDS_XZ3_list[RDS_XZ3_listnum][0]) == 0)) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();  //give reward if not "p0" (from Jeremy's code, why?)
                }
                if (RDS_XZ3_listnum < (RDS_XZ3_num -1)) {
                  RDS_XZ3_listnum++;   //move to the next reward location
                }
                else {
                  RDS_XZ3_listnum=0;  //if it's already the last reward location, reset to the first
                }
              }
            }

          
          if (RDS_XZ11_TR) {      
              randRD_flag=0;  //do not use random reward
              if ((strcmp(eventstr_1,&RDS_XZ11_TR_list[RDS_XZ11_TR_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_XZ11_TR_list[RDS_XZ11_TR_listnum][0]) == 0) ||              
                (strcmp(eventstr_4,&RDS_XZ11_TR_list[RDS_XZ11_TR_listnum][0]) == 0)) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();  //give reward if not "p0" (from Jeremy's code, why?)
                }
                if (RDS_XZ11_TR_listnum < (RDS_XZ11_TR_num -1)) {
                  RDS_XZ11_TR_listnum++;   //move to the next reward location
                }
                else {
                  RDS_XZ11_TR_listnum=0;  //if it's already the last reward location, reset to the first
                }
              }
            }
        
          if (RDS_XZ2) {      
              randRD_flag=0;  //do not use random reward
              if ((strcmp(eventstr_1,&RDS_XZ2_list[RDS_XZ2_listnum][0]) == 0) ||           //if any event string is the position that is what supposed to be the current reward position (defined in the RDS_XZ1_list)
                (strcmp(eventstr_2,&RDS_XZ2_list[RDS_XZ2_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_XZ2_list[RDS_XZ2_listnum][0]) == 0) ||              
                (strcmp(eventstr_4,&RDS_XZ2_list[RDS_XZ2_listnum][0]) == 0)) {
                //if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                //}
                //else {
                  if ((new_event_tag == 1)&&(RDS_XZ2_listnum == 0)) {   //this RDS only has one RD location. So need to add this criterion (animal enters a new location); otherwise water will be constantly draining out
                  //Some bugs occured when just use the "new location" criterion. So I now make two RD zones, but the second one is FAKE. "RDS_XZ2_listnum==0" means only give reward at the first location.
                    printRD();  //give reward if not "p0" (from Jeremy's code, why?)
                  }
                //}
                if (RDS_XZ2_listnum < (RDS_XZ2_num -1)) {
                  RDS_XZ2_listnum++;   //move to the next reward location
                }
                else {
                  RDS_XZ2_listnum=0;  //if it's already the last reward location, reset to the first
                }
              }
            }
 
          if (RDS_XZTR1) {      
              randRD_flag=0;  //do not use random reward
              if ((strcmp(eventstr_1,&RDS_XZTR1_list[RDS_XZTR1_listnum][0]) == 0) ||           //if any event string is the position that is what supposed to be the current reward position (defined in the RDS_XZ1_list)
                (strcmp(eventstr_2,&RDS_XZTR1_list[RDS_XZTR1_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_XZTR1_list[RDS_XZTR1_listnum][0]) == 0) ||              
                (strcmp(eventstr_4,&RDS_XZTR1_list[RDS_XZTR1_listnum][0]) == 0)) {
//                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
//                }
//                else {
                  printRD();
//                }
                if (RDS_XZTR1_listnum < (RDS_XZTR1_num -1)) {
                  RDS_XZTR1_listnum++;   //move to the next reward location
                }
                else {
                  RDS_XZTR1_listnum=0;  //if it's already the last reward location, reset to the first
                }
              }
            } 
             
          if (RDS_XZTR2) {      
              randRD_flag=0;  //do not use random reward
              if ((strcmp(eventstr_1,&RDS_XZTR2_list[RDS_XZTR2_listnum][0]) == 0) ||           //if any event string is the position that is what supposed to be the current reward position (defined in the RDS_XZ1_list)
                (strcmp(eventstr_2,&RDS_XZTR2_list[RDS_XZTR2_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_XZTR2_list[RDS_XZTR2_listnum][0]) == 0) ||              
                (strcmp(eventstr_4,&RDS_XZTR2_list[RDS_XZTR2_listnum][0]) == 0)) {
//                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
//                }
//                else {
                  printRD();
//                }
                if (RDS_XZTR2_listnum < (RDS_XZTR2_num -1)) {
                  RDS_XZTR2_listnum++;   //move to the next reward location
                }
                else {
                  RDS_XZTR2_listnum=0;  //if it's already the last reward location, reset to the first
                }
              }
            }
            
          if (RDS_XZTR3) {      
              randRD_flag=0;  //do not use random reward
              if ((strcmp(eventstr_1,&RDS_XZTR3_list[RDS_XZTR3_listnum][0]) == 0) ||           //if any event string is the position that is what supposed to be the current reward position (defined in the RDS_XZ1_list)
                (strcmp(eventstr_2,&RDS_XZTR3_list[RDS_XZTR3_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_XZTR3_list[RDS_XZTR3_listnum][0]) == 0) ||              
                (strcmp(eventstr_4,&RDS_XZTR3_list[RDS_XZTR3_listnum][0]) == 0)) {
//                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
//                }
//                else {
                  printRD();
//                }
                if (RDS_XZTR3_listnum < (RDS_XZTR3_num -1)) {
                  RDS_XZTR3_listnum++;   //move to the next reward location
                }
                else {
                  RDS_XZTR3_listnum=0;  //if it's already the last reward location, reset to the first
                }
              }
            }
            
            if (RDS_XZRand2) {
              if ((strcmp(eventstr_1,&RDS_XZRand2_list[index_RD2[RDS_XZRand2_listnum]][0]) == 0) ||           //if any event string is the position that is what supposed to be the current reward position (defined in the RDS_XZ1_list)
                (strcmp(eventstr_2,&RDS_XZRand2_list[index_RD2[RDS_XZRand2_listnum]][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_XZRand2_list[index_RD2[RDS_XZRand2_listnum]][0]) == 0) ||              
                (strcmp(eventstr_4,&RDS_XZRand2_list[index_RD2[RDS_XZRand2_listnum]][0]) == 0)) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();  //give reward if not "p0" (from Jeremy's code, why?)
                }
                if (RDS_XZRand2_listnum < (RDS_XZRand2_num -1)) {
                  RDS_XZRand2_listnum++;   //move to the next reward location
                }
                else {
                  RDS_XZRand2_listnum=0;  //if it's already the last reward location, reset to the first
                }
              }
            } 
            
            if (RDS_XZRand3) {
              if ((strcmp(eventstr_1,&RDS_XZRand3_list[index_RD3[RDS_XZRand3_listnum]][0]) == 0) ||           //if any event string is the position that is what supposed to be the current reward position (defined in the RDS_XZ1_list)
                (strcmp(eventstr_2,&RDS_XZRand3_list[index_RD3[RDS_XZRand3_listnum]][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_XZRand3_list[index_RD3[RDS_XZRand3_listnum]][0]) == 0) ||              
                (strcmp(eventstr_4,&RDS_XZRand3_list[index_RD3[RDS_XZRand3_listnum]][0]) == 0)) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                }
                else {
                  printRD();  //give reward if not "p0" (from Jeremy's code, why?)
                }
                if (RDS_XZRand3_listnum < (RDS_XZRand3_num -1)) {
                  RDS_XZRand3_listnum++;   //move to the next reward location
                }
                else {
                  RDS_XZRand3_listnum=0;  //if it's already the last reward location, reset to the first
                }
              }
            }

            if (RDS_XZR) {   //only random reward
              if (randRD_roll_tag) { //if entered a new zone. See previous code for updating the current event
                randRD_binomial();
                //The old random procedure is randRD_roll.
              }            
            }
            
         }
          ///////////////////////////////////////////////////////////////////////////////////////////////
        
          
          
        ////////////////////////////////////////////////          
        if (lick_RD) { // LICK-DEPENDENT REWARD SCHEDULE
        
        // For Xinyu's lick-dependent protocol in the symmetric maze
          if (RDS_XZ2) {      
            randRD_flag=0;  //do not use random reward
            if ((strcmp(eventstr_1,&RDS_XZ2_list[RDS_XZ2_listnum][0]) == 0) ||           //if any event string is the position that is what supposed to be the current reward position (defined in the RDS_XZ1_list)
              (strcmp(eventstr_2,&RDS_XZ2_list[RDS_XZ2_listnum][0]) == 0) ||
              (strcmp(eventstr_3,&RDS_XZ2_list[RDS_XZ2_listnum][0]) == 0) ||              
              (strcmp(eventstr_4,&RDS_XZ2_list[RDS_XZ2_listnum][0]) == 0)) {

              // if ((new_event_tag == 1)&&(RDS_XZ2_listnum==0)) {   //this RDS only has one RD location. So need to add this criterion (animal enters a new location); otherwise water will be constantly draining out
              // //Some bugs occured when just use the "new location" criterion. So I now make two RD zones, but the second one is FAKE. "RDS_XZ2_listnum==0" means only give reward at the first location.
              //   printRD(); 
              // }
              
              RD_zone_tag=1;  //entered a reward zone;
              
              Serial1.print(0x0c, BYTE); // clear the display
              sprintf(stringBuff, "I'm here");  
              printString1((char*)stringBuff);
              Serial1.print(0x94, BYTE); // clear the display            
              if (lk_cnt==0) {
                sprintf(stringBuff, "LK0");  
              }                
              else if (lk_cnt==1) {
                sprintf(stringBuff, "LK1");  
              }
              else if (lk_cnt==2) {
                sprintf(stringBuff, "LK2");  
              }                              
              else if (lk_cnt==3) {
                sprintf(stringBuff, "LK3");  
              }
              else if (lk_cnt==4) {
                sprintf(stringBuff, "LK4");  
              }
              else if (lk_cnt==5) {
                sprintf(stringBuff, "LK5");  
              }              
              else if (lk_cnt>5) {
                sprintf(stringBuff, "LKM");  
              }                
              printString1((char*)stringBuff);
              
              
              if ( (lk_cnt>=lk_th) && (RD_tag == 0) && (RDS_XZ2_listnum == 0) ) { // give reward when the animal accumulate a certain number of licks. 
              // Unlike the lick-dependent protocol (see commented lines above), do not use the new vent as a criterion because the animal can stay at the reward location and accumulate licks. The system should wait. 
              // No more than one reward will be delivered because RD_tag is set to 1 by reward delivery. It will only be 0 when the animal moves to a new location.
                printRD();
                
                if (RDS_XZ2_listnum < (RDS_XZ2_num -1)) { 
                //The first criterion to move to the next location: Reward has been delivered
                //The second criterion is that the animal moved out of the reward zone. See the update of eventstr_curr for this.  
                  RDS_XZ2_listnum++;   //move to the next reward location
                }
                else {
                  RDS_XZ2_listnum=0;  //if it's already the last reward location, reset to the first
                }
              
              }                           
            }
          }
        //  end of Xinyu's lick-dependent RDS
        
        
//        1B. SAME AS SECTION ABOVE BUT WITH LICK-DEPENDENCE FOR RD DELIVERY. 
          if (RDS_A1) {
            if ((strcmp(eventstr_1,&RDS_A1_list[RDS_A1_listnum][0]) == 0) ||
              (strcmp(eventstr_2,&RDS_A1_list[RDS_A1_listnum][0]) == 0)) {
              if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
              }
              else {
                printRD();
              }
              if (RDS_A1_listnum < (RDS_A1_num -1)) {
                RDS_A1_listnum++;
              }
              else {
                RDS_A1_listnum=0;
              }
            }
          }
         
          if (RDS_A2) {
            if (speed_th_tag) {
              speed_th_curr = 5000;
            }
            if (RD_cnt < 100) {
              if ((strcmp(eventstr_1,&RDS_A2_list[RDS_A2_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_A2_list[RDS_A2_listnum][0]) == 0)) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  lk_tag=0;
                }
                else {
                  printRD();
                  lk_tag=0;
                }
                if (RDS_A2_listnum < (RDS_A2_num -1)) {
                  RDS_A2_listnum++;
                }
                else {
                  RDS_A2_listnum=0;
                }
              }
            }
            if (RD_cnt < 150) {
              if ((strcmp(eventstr_1,&RDS_A2_list[RDS_A2_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_A2_list[RDS_A2_listnum][0]) == 0)) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  lk_tag=0;
                }
                else if (lk_tag) {
                  printRD();
                  lk_tag=0;
                }
                if (RDS_A2_listnum < (RDS_A2_num -1)) {
                  RDS_A2_listnum++;
                }
                else {
                  RDS_A2_listnum=0;
                }
              }
            }
            else {
              RDS_A2 = 0;
              RDS_A3 = 1;
            }
          } 
          
          if (RDS_A3) {
            if (RD_cnt < 180) {
              if (speed_th_tag) {
              speed_th_curr = 4800;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 210) {
              if (speed_th_tag) {
                speed_th_curr = 4600;
              }                
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 240) {
              if (speed_th_tag) {
                speed_th_curr = 4400;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 270) {
              if (speed_th_tag) {
                speed_th_curr = 4200;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 300) {
              if (speed_th_tag) {
                speed_th_curr = 4000;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 330) {
              if (speed_th_tag) {
                speed_th_curr = 3900;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 360) {
              if (speed_th_tag) {
                speed_th_curr = 3800;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 390) {
              if (speed_th_tag) {
                speed_th_curr = 3700;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 420) {
              if (speed_th_tag) {
                speed_th_curr = 3600;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 450) {
              if (speed_th_tag) {
                speed_th_curr = 3500;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 480) {
              if (speed_th_tag) {
                speed_th_curr = 3400;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 500) {
              if (speed_th_tag) {
                speed_th_curr = 3300;
              }
              if ((strcmp(eventstr_1,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A3_list[RDS_A3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A3_list[RDS_A3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A3_list[RDS_A3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A3_listnum < (RDS_A3_num -1)) {
                  RDS_A3_listnum++;
                }
                else {
                  RDS_A3_listnum=0;
                }
              }
            }
            else {
              RDS_A3 = 0;
              RDS_A4 = 1;
              training = 0;
              uart1_put(0x80); // row 0, col 0
              Serial1.print(0x0c, BYTE); // clear the display
              sprintf(stringBuff, "* RDS_A4 *"); // 
              printString1((char*)stringBuff);
            }
          }
           
          if (RDS_A4) {
            if (RD_cnt < 540) {
              if (speed_th_tag) {
              speed_th_curr = 3100;
              }
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
              if (randRD_flag) {
                randRD_roll();
              }
            }
            else if (RD_cnt < 570) {
              if (speed_th_tag) {
                speed_th_curr = 3000;
              }
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                    (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                    (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                    (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 600) {
              if (speed_th_tag) {
                speed_th_curr = 2900; 
              }
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 630) {
              if (speed_th_tag) {
                speed_th_curr = 2800; 
              }
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 660) {
              if (speed_th_tag) {
                speed_th_curr = 2800; 
              }  
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 690) {
              if (speed_th_tag) {
                speed_th_curr = 2700;
              }
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 720) {
              if (speed_th_tag) {
                speed_th_curr = 2600;
              }
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
            }
            else {
              if (speed_th_tag) {
                speed_th_curr = 2500;
              }
              if ((strcmp(eventstr_1,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_A4_list[RDS_A4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_A4_list[RDS_A4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_A4_list[RDS_A4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_A4_listnum < (RDS_A4_num -1)) {
                  RDS_A4_listnum++;
                }
                else {
                  RDS_A4_listnum=0;
                }
              }
            }
          }
              
          if (RDS_A14) {
            if ((strcmp(eventstr_1,&RDS_A14_list[RDS_A14_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_A14_list[RDS_A14_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_A14_list[RDS_A14_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_A14_list[RDS_A14_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  lk_tag=0;
                }
                else if (lk_tag) {
                  printRD();
                  lk_tag = 0;
                }
              }
              if (RDS_A14_listnum < (RDS_A14_num -1)) {
                RDS_A14_listnum++;
              }
              else {
                RDS_A14_listnum=0;
              }
            }
          }
             
          
//        2. State codes for rat position-contingent reward schedule in L-track (RDS_B)
          if (RDS_B1) {
            if ((strcmp(eventstr_1,&RDS_B1_list[RDS_B1_listnum][0]) == 0) ||
              (strcmp(eventstr_2,&RDS_B1_list[RDS_B1_listnum][0]) == 0)) {
              if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
              }
              else {
                printRD();
              }
              if (RDS_B1_listnum < (RDS_B1_num -1)) {
                RDS_B1_listnum++;
              }
              else {
                RDS_B1_listnum=0;
              }
            }
          }
         
          if (RDS_B2) {
            if (speed_th_tag) {
              speed_th_curr = 5000;
            }
            if (RD_cnt < 100) {
              if ((strcmp(eventstr_1,&RDS_B2_list[RDS_B2_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_B2_list[RDS_B2_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_B2_list[RDS_B2_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_B2_list[RDS_B2_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B2_listnum < (RDS_B2_num -1)) {
                  RDS_B2_listnum++;
                }
                else {
                  RDS_B2_listnum=0;
                }
              }
            }
            if (RD_cnt < 150) {
              if ((strcmp(eventstr_1,&RDS_B2_list[RDS_B2_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_B2_list[RDS_B2_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_B2_list[RDS_B2_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_B2_list[RDS_B2_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B2_listnum < (RDS_B2_num -1)) {
                  RDS_B2_listnum++;
                }
                else {
                  RDS_B2_listnum=0;
                }
              }
            }
            else {
             RDS_B2 = 0;
             RDS_B3 = 1;            
            }
          }
                 
          if (RDS_B3) {
            if (RD_cnt < 180) {
              if (speed_th_tag) {
                speed_th_curr = 4800;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 210) {
              if (speed_th_tag) {
                speed_th_curr = 4600;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 240) {
              if (speed_th_tag) {
                speed_th_curr = 4400;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 270) {
              if (speed_th_tag) {
                speed_th_curr = 4200;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 300) {
              if (speed_th_tag) {
                speed_th_curr = 4000;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 330) {
              if (speed_th_tag) {
                speed_th_curr = 3900;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 360) {
              if (speed_th_tag) {
                speed_th_curr = 3800;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 390) {
              if (speed_th_tag) {
                speed_th_curr = 3700;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 420) {
              if (speed_th_tag) {
                speed_th_curr = 3600;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 450) {
              if (speed_th_tag) {
                speed_th_curr = 3500;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 480) {
              if (speed_th_tag) {
                speed_th_curr = 3400;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 500) {
              if (speed_th_tag) {
                speed_th_curr = 3300;
              }
              if ((strcmp(eventstr_1,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B3_list[RDS_B3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B3_list[RDS_B3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B3_list[RDS_B3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B3_listnum < (RDS_B3_num -1)) {
                  RDS_B3_listnum++;
                }
                else {
                  RDS_B3_listnum=0;
                }
              }
            }
            else {
              RDS_B3 = 0;
              RDS_B4 = 1;
              training = 0;
              uart1_put(0x80); // row 0, col 0
              Serial1.print(0x0c, BYTE); // clear the display
              sprintf(stringBuff, "* RDS_B4 *"); // 
              printString1((char*)stringBuff);
            }
          }
           
          if (RDS_B4) {
            if (RD_cnt < 540) {
              if (speed_th_tag) {
                speed_th_curr = 3200;
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
              if (randRD_flag) {
                randRD_roll();
              }
            }
            else if (RD_cnt < 570) {
              if (speed_th_tag) {
                speed_th_curr = 3100;
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                    (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                    (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                    (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 600) {
              if (speed_th_tag) {
                speed_th_curr = 3000; 
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 630) {
              if (speed_th_tag) {
                speed_th_curr = 2900; 
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 660) {
              if (speed_th_tag) {
                speed_th_curr = 2800; 
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 690) {
              if (speed_th_tag) {
                speed_th_curr = 2700;
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 720) {
              if (speed_th_tag) {
                speed_th_curr = 2600;
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
            }
            else {
              if (speed_th_tag) {
                speed_th_curr = 2500;
              }
              if ((strcmp(eventstr_1,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_B4_list[RDS_B4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_B4_list[RDS_B4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_B4_list[RDS_B4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_B4_listnum < (RDS_B4_num -1)) {
                  RDS_B4_listnum++;
                }
                else {
                  RDS_B4_listnum=0;
                }
              }
            }
          }
          
          if (RDS_B14) {
            if ((strcmp(eventstr_1,&RDS_B14_list[RDS_B14_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_B14_list[RDS_B14_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_B14_list[RDS_B14_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_B14_list[RDS_B14_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
              if (RDS_B14_listnum < (RDS_B14_num -1)) {
                RDS_B14_listnum++;
              }
              else {
                RDS_B14_listnum=0;
              }
            }
          }


//        3. State codes for rat position-contingent reward schedule in fig8-track (RDS_C)
          if (RDS_C1) {
            if ((strcmp(eventstr_1,&RDS_C1_list[RDS_C1_listnum][0]) == 0) ||
              (strcmp(eventstr_2,&RDS_C1_list[RDS_C1_listnum][0]) == 0)) {
              if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
              }
              else {
                printRD();
              }
              if (RDS_C1_listnum < (RDS_C1_num -1)) {
                RDS_C1_listnum++;
              }
              else {
                RDS_C1_listnum=0;
              }
            }
          }
  
          if (RDS_C2) {
            if (speed_th_tag) {
              speed_th_curr = 5000;
            }
            if (RD_cnt < 100) {
              if ((strcmp(eventstr_1,&RDS_C2_list[RDS_C2_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_C2_list[RDS_C2_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_C2_list[RDS_C2_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_C2_list[RDS_C2_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lapMarker();
                    lk_tag=0;
                  }
                  else {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C2_listnum < (RDS_C2_num -1)) {
                  RDS_C2_listnum++;
                }
                else {
                  RDS_C2_listnum=0;
                }
              }
            }
            if (RD_cnt < 150) {
              if ((strcmp(eventstr_1,&RDS_C2_list[RDS_C2_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_C2_list[RDS_C2_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_C2_list[RDS_C2_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_C2_list[RDS_C2_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lapMarker();
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C2_listnum < (RDS_C2_num -1)) {
                  RDS_C2_listnum++;
                }
                else {
                  RDS_C2_listnum=0;
                }
              }
            }
            else {
              RDS_C2 = 0;
              RDS_C3 = 1;
            }
          }           
                   
          if (RDS_C3) {
            if (RD_cnt < 180) {
              if (speed_th_tag) {
                speed_th_curr = 4800;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 210) {
              if (speed_th_tag) {
                speed_th_curr = 4600;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 240) {
              if (speed_th_tag) {
                speed_th_curr = 4400;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 270) {
              if (speed_th_tag) {
                speed_th_curr = 4200;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 300) {
              if (speed_th_tag) {
                speed_th_curr = 4000;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 330) {
              if (speed_th_tag) {
                speed_th_curr = 3900;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 360) {
              if (speed_th_tag) {
                speed_th_curr = 3800;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 390) {
              if (speed_th_tag) {
                speed_th_curr = 3700;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 420) {
              if (speed_th_tag) {
                speed_th_curr = 3600;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 450) {
              if (speed_th_tag) {
                speed_th_curr = 3500;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 480) {
              if (speed_th_tag) {
                speed_th_curr = 3400;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 500) {
              if (speed_th_tag) {
                speed_th_curr = 3300;
              }
              if ((strcmp(eventstr_1,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C3_list[RDS_C3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C3_list[RDS_C3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C3_list[RDS_C3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C3_listnum < (RDS_C3_num -1)) {
                  RDS_C3_listnum++;
                }
                else {
                  RDS_C3_listnum=0;
                }
              }
            }
            else {
              RDS_C3 = 0;
              RDS_C4 = 1;
              training = 0;
              uart1_put(0x80); // row 0, col 0
              Serial1.print(0x0c, BYTE); // clear the display
              sprintf(stringBuff, "* RDS_C4 *"); // 
              printString1((char*)stringBuff);
            }
          }
           
          if (RDS_C4) {
            if (RD_cnt < 540) {
              if (speed_th_tag) {
                speed_th_curr = 3200;
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  lk_tag=0;
                }
                else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
              if (randRD_flag) {
                randRD_roll();
              }
            }
            else if (RD_cnt < 570) {
              if (speed_th_tag) {
                speed_th_curr = 3100;
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                    (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                    (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                    (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 600) {
              if (speed_th_tag) {
                speed_th_curr = 3000; 
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 630) {
              if (speed_th_tag) {
                speed_th_curr = 2900; 
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 660) {
              if (speed_th_tag) {
                speed_th_curr = 2800; 
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 690) {
              if (speed_th_tag) {
                speed_th_curr = 2700;
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 720) {
              if (speed_th_tag) {
                speed_th_curr = 2600;
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
            }
            else {
              if (speed_th_tag) {
                speed_th_curr = 2500;
              }
              if ((strcmp(eventstr_1,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_C4_list[RDS_C4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_C4_list[RDS_C4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_C4_list[RDS_C4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_C4_listnum < (RDS_C4_num -1)) {
                  RDS_C4_listnum++;
                }
                else {
                  RDS_C4_listnum=0;
                }
              }
            }
          }
          
          if (RDS_C14) {
            if ((strcmp(eventstr_1,&RDS_C14_list[RDS_C14_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_C14_list[RDS_C14_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_C14_list[RDS_C14_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_C14_list[RDS_C14_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
              if (RDS_C14_listnum < (RDS_C14_num -1)) {
                RDS_C14_listnum++;
              }
              else {
                RDS_C14_listnum=0;
              }
            }
          }


//        4. State codes for rat position-contingent reward schedule in oval-track (RDS_D)
          if (RDS_D1) {
            if ((strcmp(eventstr_1,&RDS_D1_list[RDS_D1_listnum][0]) == 0) ||
              (strcmp(eventstr_2,&RDS_D1_list[RDS_D1_listnum][0]) == 0) ||
              (strcmp(eventstr_3,&RDS_D1_list[RDS_D1_listnum][0]) == 0) || 
              (strcmp(eventstr_4,&RDS_D1_list[RDS_D1_listnum][0]) == 0)) {
              if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
              }
              else {
                printRD();
              }
              if (RDS_D1_listnum < (RDS_D1_num -1)) {
                RDS_D1_listnum++;
              }
              else {
                RDS_D1_listnum=0;
              }
            }
          }
         
          if (RDS_D2) {
            if (speed_th_tag) {
              speed_th_curr = 5000;
            }
            if (RD_cnt < 100) {
              if ((strcmp(eventstr_1,&RDS_D2_list[RDS_D2_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_D2_list[RDS_D2_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_D2_list[RDS_D2_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_D2_list[RDS_D2_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D2_listnum < (RDS_D2_num -1)) {
                  RDS_D2_listnum++;
                }
                else {
                  RDS_D2_listnum=0;
                }
              }
            }
            if (RD_cnt < 150) {
              if ((strcmp(eventstr_1,&RDS_D2_list[RDS_D2_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_D2_list[RDS_D2_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_D2_list[RDS_D2_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_D2_list[RDS_D2_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D2_listnum < (RDS_D2_num -1)) {
                  RDS_D2_listnum++;
                }
                else {
                  RDS_D2_listnum=0;
                }
              }
            }
            else {
              RDS_D2 = 0;
              RDS_D3 = 1;
            }
          }           
                   
          if (RDS_D3) {
            if (RD_cnt < 180) {
              if (speed_th_tag) {
                speed_th_curr = 4800;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 210) {
              if (speed_th_tag) {
                speed_th_curr = 4600;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 240) {
              if (speed_th_tag) {
                speed_th_curr = 4400;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 270) {
              if (speed_th_tag) {
                speed_th_curr = 4200;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 300) {
              if (speed_th_tag) {
                speed_th_curr = 4000;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 330) {
              if (speed_th_tag) {
                speed_th_curr = 3900;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 360) {
              if (speed_th_tag) {
                speed_th_curr = 3800;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 390) {
              if (speed_th_tag) {
                speed_th_curr = 3700;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 420) {
              if (speed_th_tag) {
                speed_th_curr = 3600;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 450) {
              if (speed_th_tag) {
                speed_th_curr = 3500;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 480) {
              if (speed_th_tag) {
                speed_th_curr = 3400;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else if (RD_cnt < 500) {
              if (speed_th_tag) {
                speed_th_curr = 3300;
              }
              if ((strcmp(eventstr_1,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D3_list[RDS_D3_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D3_list[RDS_D3_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D3_list[RDS_D3_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D3_listnum < (RDS_D3_num -1)) {
                  RDS_D3_listnum++;
                }
                else {
                  RDS_D3_listnum=0;
                }
              }
            }
            else {
              RDS_D3 = 0;
              RDS_D4 = 1;
              training = 0;
              uart1_put(0x80); // row 0, col 0
              Serial1.print(0x0c, BYTE); // clear the display
              sprintf(stringBuff, "* RDS_D4 *"); // 
              printString1((char*)stringBuff);
            }
          }
           
          if (RDS_D4) {
            if (RD_cnt < 540) {
              if (speed_th_tag) {
                speed_th_curr = 3200;
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
              if (randRD_flag) {
                randRD_roll();
              }
            }
            else if (RD_cnt < 570) {
              if (speed_th_tag) {
                speed_th_curr = 3100;
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                    (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                    (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                    (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }  
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 600) {
              if (speed_th_tag) {
                speed_th_curr = 3000; 
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 630) {
              if (speed_th_tag) {
                speed_th_curr = 2900; 
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 660) {
              if (speed_th_tag) {
                speed_th_curr = 2800; 
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 690) {
              if (speed_th_tag) {
                speed_th_curr = 2700;
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
            }
            else if (RD_cnt < 720) {
              if (speed_th_tag) {
                speed_th_curr = 2600;
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
            }
            else {
              if (speed_th_tag) {
                speed_th_curr = 2500;
              }
              if ((strcmp(eventstr_1,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_2,&RDS_D4_list[RDS_D4_listnum][0]) == 0) ||
                  (strcmp(eventstr_3,&RDS_D4_list[RDS_D4_listnum][0]) == 0) || 
                  (strcmp(eventstr_4,&RDS_D4_list[RDS_D4_listnum][0]) == 0)) {
                if (avg_speed < speed_th) {
                  if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                    lk_tag=0;
                  }
                  else if (lk_tag) {
                    printRD();
                    lk_tag = 0;
                  }
                }
                if (RDS_D4_listnum < (RDS_D4_num -1)) {
                  RDS_D4_listnum++;
                }
                else {
                  RDS_D4_listnum=0;
                }
              }
            }
          }
          
          if (RDS_D14) {
            if ((strcmp(eventstr_1,&RDS_D14_list[RDS_D14_listnum][0]) == 0) ||
                (strcmp(eventstr_2,&RDS_D14_list[RDS_D14_listnum][0]) == 0) ||
                (strcmp(eventstr_3,&RDS_D14_list[RDS_D14_listnum][0]) == 0) || 
                (strcmp(eventstr_4,&RDS_D14_list[RDS_D14_listnum][0]) == 0)) {
              if (avg_speed < speed_th) {
                if ((strcmp(eventstr_1,"p0") ==0) || (strcmp(eventstr_2,"p0") ==0)) {
                  lk_tag=0;
                }
                else if (lk_tag) {
                  printRD();
                  lk_tag = 0;
                }
              }
              if (RDS_D14_listnum < (RDS_D14_num -1)) {
                RDS_D14_listnum++;
              }
              else {
                RDS_D14_listnum=0;
              }
            }
          }
        }

//        4. State code for closed-loop analog gain control auditory stim box (AGC-ASB)
//        Auditory stimulation schedule (ASS)
          if (ASS_A) {
            if (avg_speed < 100) {
              dac2Write(0,0);
            }
            else if (avg_speed < 200) {
              dac2Write(0,512);
            }
            else if (avg_speed < 300) {
              dac2Write(0,1024);
            }
            else if (avg_speed < 400) {
              dac2Write(0,2048);
            }
            else if (avg_speed < 600) {
              dac2Write(0,4096);
            }
            else if (avg_speed < 800) {
              dac2Write(0,8192);
            }
            else if (avg_speed < 1200) {
              dac2Write(0,16348);
            }
            else if (avg_speed < 1600) {
              dac2Write(0,32768);
            }           
            else if (avg_speed < 2000) {
              dac2Write(0,49152);
            }
            else if (avg_speed < 2400) {
              dac2Write(0,60000);
            }
            else {
              dac2Write(0,65000);
            }            
          }
          
   
          // State code for TRAINING button. signal sent from Matlab gui. On=1, Off=0; default Off
          if (((training) && (RDS_A1))) { // regulate speed threshold based on RD_cnt
            if (RD_cnt < 10) { // gives some free RDs just for moving at the beggining
              speed_th_curr = 400;
              RDS_A1 = 1;
            }  
            else if (RD_cnt < 20) {
              speed_th_curr = 800; 
            }
            else if (RD_cnt < 30) {
              speed_th_curr = 1200; 
            }
            else if (RD_cnt < 40) {
              speed_th_curr = 1600; 
            }
            else if (RD_cnt < 50) {
              speed_th_curr = 2000; 
            }
            else if (RD_cnt < 60) { // last free in A1 but no more free RDs
              speed_th_curr = 5000; // out of reach, practically off
            }
            else if (RD_cnt < 150) {
              RDS_A1 = 0;
              RDS_A2 = 1; // increase difficulty of reward schedule
            }
            else if (RD_cnt < 600) {
              RDS_A1 = 0;
              RDS_A2 = 0;
              RDS_A3 = 1; // increase difficulty of reward schedule
            }
            else {
              RDS_A1 = 0;
              RDS_A2 = 0;
              RDS_A3 = 0; 
              RDS_A4 = 1; // increase difficulty of reward schedule
            }
            
            if (avg_speed < 200) {
              speed_sent = 0
              ;
            }
            else if ((avg_speed > speed_th) && (speed_sent ==0)) {
              printRD();
              speed_sent = 1;
            }
          }
          
          
          if ((training) && (RDS_B1)) {
            if (RD_cnt < 10) { 
              speed_th_curr = 400;
              RDS_B1 = 1;
            }  
            else if (RD_cnt < 20) {
              speed_th_curr = 800; 
            }
            else if (RD_cnt < 30) {
              speed_th_curr = 1200; 
            }
            else if (RD_cnt < 40) {
              speed_th_curr = 1600; 
            }
            else if (RD_cnt < 50) {
              speed_th_curr = 2000; 
            }
            else if (RD_cnt < 60) {
              RDS_B1 = 1;
              speed_th_curr = 5000; // out of reach, practically off
            }
            else if (RD_cnt < 150) {
              RDS_B1 = 0;
              RDS_B2 = 1; // increase difficulty of reward schedule
            }
            else if (RD_cnt < 600) {
              RDS_B1 = 0;
              RDS_B2 = 0;
              RDS_B3 = 1; // increase difficulty of reward schedule
            }
            else {
              RDS_B1 = 0;
              RDS_B2 = 0;
              RDS_B3 = 0; 
              RDS_B4 = 1; // increase difficulty of reward schedule
            }
            
            if (avg_speed < 200) {
              speed_sent = 0;
            }
            else if ((avg_speed > speed_th) && (speed_sent ==0)) {
              printRD();
              speed_sent = 1;
            }
          }
          
          
          if ((training) && (RDS_C1)) {
            if (RD_cnt < 10) { 
              speed_th_curr = 400;
              RDS_C1 = 1;
            }  
            else if (RD_cnt < 20) {
              speed_th_curr = 800; 
            }
            else if (RD_cnt < 30) {
              speed_th_curr = 1200; 
            }
            else if (RD_cnt < 40) {
              speed_th_curr = 1600; 
            }
            else if (RD_cnt < 50) {
              speed_th_curr = 2000; 
            }
            else if (RD_cnt < 60) {
              RDS_C1 = 1;
              speed_th_curr = 5000; // out of reach, practically off
            }
            else if (RD_cnt < 150) {
              RDS_C1 = 0;
              RDS_C2 = 1; // increase difficulty of reward schedule
            }
            else if (RD_cnt < 600) {
              RDS_C1 = 0;
              RDS_C2 = 0;
              RDS_C3 = 1; // increase difficulty of reward schedule
            }
            else {
              RDS_C1 = 0;
              RDS_C2 = 0;
              RDS_C3 = 0; 
              RDS_C4 = 1; // increase difficulty of reward schedule
            }
            
            if (avg_speed < 200) {
              speed_sent = 0;
            }
            else if ((avg_speed > speed_th) && (speed_sent ==0)) {
              printRD();
              speed_sent = 1;
            }
          }
          
          if ((training) && (RDS_D1)) {
            if (RD_cnt < 10) { 
              speed_th_curr = 400;
              RDS_D1 = 1;
            }  
            else if (RD_cnt < 20) {
              speed_th_curr = 800; 
            }
            else if (RD_cnt < 30) {
              speed_th_curr = 1200; 
            }
            else if (RD_cnt < 40) {
              speed_th_curr = 1600; 
            }
            else if (RD_cnt < 50) {
              speed_th_curr = 2000; 
            }
            else if (RD_cnt < 60) {
              RDS_D1 = 1;
              speed_th_curr = 5000; // out of reach, practically off
            }
            else if (RD_cnt < 150) {
              RDS_D1 = 0;
              RDS_D2 = 1; // increase difficulty of reward schedule
            }
            else if (RD_cnt < 600) {
              RDS_D1 = 0;
              RDS_D2 = 0;
              RDS_D3 = 1; // increase difficulty of reward schedule
            }
            else {
              RDS_D1 = 0;
              RDS_D2 = 0;
              RDS_D3 = 0; 
              RDS_D4 = 1; // increase difficulty of reward schedule
            }
            if (avg_speed < 200) {
              speed_sent = 0;
            }
            else if ((avg_speed > speed_th) && (speed_sent ==0)) {
              printRD();
              speed_sent = 1;
            }
          }
          
          if ((training) && (RDS_E1)) {
            if (RD_cnt < 10) { 
              speed_th_curr = 400;
              RDS_E1 = 1;
            }  
            else if (RD_cnt < 20) {
              speed_th_curr = 800; 
            }
            else if (RD_cnt < 30) {
              speed_th_curr = 1200; 
            }
            else if (RD_cnt < 40) {
              speed_th_curr = 1600; 
            }
            else if (RD_cnt < 50) {
              speed_th_curr = 2000; 
            }
            else if (RD_cnt < 60) {
              RDS_E1 = 1;
              speed_th_curr = 5000; // out of reach, practically off
            }
            else if (RD_cnt < 150) {
              RDS_E1 = 0;
              RDS_E2 = 1; // increase difficulty of reward schedule
            }
            else if (RD_cnt < 600) {
              RDS_E1 = 0;
              RDS_E2 = 0;
              RDS_E3 = 1; // increase difficulty of reward schedule
            }
            else {
              RDS_E1 = 0;
              RDS_E2 = 0;
              RDS_E3 = 0; 
              RDS_E4 = 1; // increase difficulty of reward schedule
            }
            
            if (avg_speed < 200) {
              speed_sent = 0;
            }
            else if ((avg_speed > speed_th) && (speed_sent ==0)) {
              printRD();
              speed_sent = 1;
            }
          }
        
        
          // more Training state code for rollMax. As RD_cnt increases, lower the rollMax - see void randRD_roll        
          if ((training) && ((RDS_E4) ||(RDS_D4) || (RDS_C4) || (RDS_B4) || (RDS_A4))) {
            if (RD_cnt < 20) {
              rollMax = 10;              
            }
            else if (RD_cnt < 40) {
              rollMax = 9;              
            }
            else if (RD_cnt < 60) {
              rollMax = 8;              
            }
            else if (RD_cnt < 80) {
              rollMax = 7;              
            }
            else if (RD_cnt < 100) {
              rollMax = 6;              
            }
            else if (RD_cnt < 150) {
              rollMax = 5;              
            }
            else if (RD_cnt < 300) {
              rollMax = 4;              
            }
            else if (RD_cnt < 600) {
              rollMax = 3;              
            }
            else if (RD_cnt < 1000) {
              rollMax = 2; 
            }
            else if (RD_cnt < 2000) {
              rollMax = 1; 
            }            
            else {
              rollMax = 0;
            }
          }
        
        }
        vr_idx = 0; // loop ending, reset vr_idx to 0
      }
      else if ((c >= ' ') && (vr_idx < sizeof(vrbuffer) - 1)) {
        lastVrbuffer[vr_idx] = c;
        vrbuffer[vr_idx++] = c;
      }
    } // END serial loop for VR communication


/*
////////////////////////////////////////////////////////////////////////////
// TTL-OUTPUT TIMER FUNCTION FOR INTRACELLULAR STIMULATION IN THE "stimZone"
////////////////////////////////////////////////////////////////////////////
    // stimZone timer function. 
    // if global stimZone_flag=1(ON), 
    // AND when in the stimZone (see void stimZone), stim_tag_i=1
    if (stimZone_flag) {
      stim_now = micros(); // reset timer loop for turning ON stimulus
      if (stim_tag_i && (stim_now - stim_start > stim_i)) { // if interval has passed, turn ON
        stim_start = stim_now; // reset timer start
        stim_tag_d = 1; // turn on tag to start counting pulse duration
        digitalWrite(DIGITAL6,HIGH); // stim ON
      }
      stim_timer = micros(); // reset timer loop for pulse duration
      if ((stim_tag_d) && (stim_timer - stim_start > stim_d)) { // if duration has passed, turn OFF
        stim_tag_d = 0; // stop checking timer
        digitalWrite(DIGITAL6,LOW); // turn OFF pulse
      }
    }

*/    
    
///////////////////////////////////
// DIGITAL INPUT-SPECIFIC RESPONSES 
///////////////////////////////////

    if (digitalRead(DIGITAL2) == HIGH) {
      if (ff_sent == 0) {
        printTimestamp();
        sprintf(stringBuff, "FF"); // indicate a Frame Flash event
        printlnString0((char*)stringBuff);
        ff_sent = 1;
      }
    } 
    else {
      ff_sent = 0;
    }
    
    if (digitalRead(DIGITAL3) == HIGH) {
      if (pm_sent == 0) {
        printTimestamp();
        sprintf(stringBuff, "PM"); // indicate a Protocol Marker event, attached to string string
        printlnString0((char*)stringBuff);
        pm_sent = 1;
      }
    } 
    else {
      pm_sent = 0;
    }
    
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Lick detection triggered by the rising edge of digital 5. When it's reviously LOW (lk_sent=0) but now HIGH, lk_tag was set to 1. 
// Falling edge does not set lk_tag to 0. lk_tag is reset to 0 by reward delivery (see lick-dependent RDS). 

    if (digitalRead(DIGITAL5) == HIGH) {
      if (lk_sent == 0) {
        printTimestamp();
        sprintf(stringBuff, "LK"); // indicate a lick event string
        printlnString0((char*)stringBuff);  //to the PC
        lk_sent = 1; 
        lk_tag = 1;
        
        lk_cnt ++;  //update th lick count. The lick count is reset to 0 when the animal move to a new location or a reward is delivered.           
        
        Serial1.print(0x99, BYTE); //move the cursur to position 5 (counted from 0), the second line.
        if (lk_cnt==0) {
             sprintf(stringBuff, "0"); //show the lick count;
        }
        else if (lk_cnt==1){
          sprintf(stringBuff, "1");  
        }
        else if (lk_cnt==2) {
          sprintf(stringBuff, "2");
        }
        else if (lk_cnt==3) {
          sprintf(stringBuff, "3");
        }
        else if (lk_cnt>3) {
          sprintf(stringBuff, "M");
        }          
        printString1((char*)stringBuff);
        
      }
    } 
    else {
      lk_sent = 0;
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // When a spike is detected by meeting the AP threshold detection criteria, then output a Threshold event string    
    if (thDetect == 1) {
      printTimestamp();
      sprintf(stringBuff, "TH"); // indicate threshold event string (spike detection)
      printlnString0((char*)stringBuff);
      thDetect = 0;
    }


//////////////////////////////////////////////////////////
// FINAL CHECKS - UPDATE BEHAVIORAL SCORES - PRINT CHANGES
//////////////////////////////////////////////////////////

    // RD_score = avg number of RDs delivered per lap as % of min_RD
    // lap_loop = num of laps to use in average
    // min_RD is min num of RDs that can be received per lap - given in zone RDS_x
    if (lap_cnt >= lap_loop) {
      RD_score = (RD_cnt_curr)/(min_RD*lap_loop); // calculate RD_score
      RD_cnt_curr = (RD_cnt_curr) - (min_RD * RD_score); // update RD_cnt_curr (substract a one-lap avg value from mean)
      lap_cnt = lap_cnt-1; // update lap_cnt
      printTimestamp();
      sprintf(stringBuff, "RD_score,");
      printString0((char*)stringBuff);
      printValue0_U32(RD_score);
      printNewline0();
    }
    
    // check to see if speed_th has changed. if so, update speed_th and print it
    if (speed_th != speed_th_curr) {
      speed_th = speed_th_curr;
      printTimestamp();
      sprintf(stringBuff, "speed_th,");
      printString0((char*)stringBuff);
      printValue0_U32(speed_th);
      printNewline0();
    }

    // check if min_RD_curr is set to current min_RD value, if not, set and print 
    if (min_RD_curr != min_RD) {
      min_RD_curr = min_RD;
      printTimestamp();
      sprintf(stringBuff, "minRD,");
      printString0((char*)stringBuff);
      printValue0_U32(min_RD_curr);
      printNewline0();
    }

    // check if rollMax_curr is set to current rollMax value, if not, set and print 
    if (rollMax_curr != rollMax) {
      rollMax_curr = rollMax;
      printTimestamp();
      sprintf(stringBuff, "rollMax,");
      printString0((char*)stringBuff);
      printValue0_U32(rollMax_curr);
      printNewline0();
    }


/////////////////////////////////////////////////////////////////////
//  THIS IS THE LAST SECTION OF CODE, CALCULATES PROCESSING LOOP TIME 
/////////////////////////////////////////////////////////////////////

    // get the time stamp
    loopTime_us = getTimestamp() - ts;
    // see if this is the longest ever
    if (loopTime_us > loopTimeMax) {
      loopTimeMax = loopTime_us;
      printTimestamp();
      sprintf(stringBuff, "MaxLoopTime,");
      printString0((char*)stringBuff);
      printValue0_U32(loopTimeMax);
      printNewline0();
    }


  } // END for (;;) loop 
} // END void loop()


