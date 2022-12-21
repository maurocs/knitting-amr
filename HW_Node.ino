#include <AccelStepper.h>
#include <Arduino_MachineControl.h>
#include <CAN.h>
#include "Wire.h"
using namespace machinecontrol;

#define sensor_pin DIN_READ_CH_PIN_00
#define green_pin DIN_READ_CH_PIN_01
#define estop_pin DIN_READ_CH_PIN_02

#define enbl_motors 0
#define tray_A 1
#define tray_B 2
#define blade_A 3
#define blade_B 4
#define buzzer 5

const long canRate = 1000000;

union cmd {
  long value;
  uint8_t bytes[sizeof(long)];
};

typedef enum e_State {
    STATE_START = 0,
    STATE_READ_RS232,
    STATE_CHECK_IO,
    STATE_READ_CAN,
    STATE_DROP_BLADE,
    STATE_LIFT_BLADE,
    STATE_ROTATE_TRAY,
    STATE_SEND_RS232 
} eState;

long encoderL_count;
long encoderR_count;
long voltage;
long soc;

int motor_command_to = 500;
int rs232_freq = 20;
int non_crit_values_freq = 1000;
long last_non_crit = millis();
long last_rs232_time = millis();
long last_mot_cmd_time = millis();
long last_pick = millis();

int incomingByte = 0;

unsigned int controllerID = 1537;
unsigned int cpuID = 1539;

bool can_ok = false;
bool rs232_ok = false;

char RS232_str[10];

mbed::CANMessage in_msg;

const byte numChars = 30;
char receivedChars[numChars];
boolean newData = false;

char msg_msg[32];

byte flags_msg[4]     = {0x40,0x21,0x12,0x00};
byte encoderL_msg[4]  = {0x40,0x21,0x04,0x01};
byte encoderR_msg[4]  = {0x40,0x21,0x04,0x02};
byte motorL_msg[4]    = {0x40,0x21,0x22,0x01};
byte motorR_msg[4]    = {0x40,0x21,0x22,0x02};
byte voltage_msg[4]   = {0x40,0x21,0x0D,0x00};
byte soc_msg[4]       = {0x40,0x21,0x3A,0x00};
byte speedL_comm[4]    = {0x20,0x20,0x00,0x01};
byte speedR_comm[4]    = {0x20,0x20,0x00,0x02};

union cmd flags_res     = {value:1};
union cmd encoderL_res  = {value:2};
union cmd encoderR_res  = {value:3};
union cmd motorL_res    = {value:4};
union cmd motorR_res    = {value:5};
union cmd voltage_res   = {value:6};
union cmd soc_res       = {value:7};
union cmd state_res     = {value:0};

union cmd speedL     = {value : 33};
union cmd speedR     = {value : 33};
union cmd soft_estop = {value : 33};
union cmd cpu_status = {value : 33};
union cmd ctrl_bytes = {value : 33};

mbed::CANMessage create_msg_obj(byte _data[], cmd _value = {value:0}) {
  mbed::CANMessage msg;
  msg.id = 1537;
  msg.len = 0x08;
  msg.data[0] = _data[0];
  msg.data[1] = _data[2];
  msg.data[2] = _data[1];
  msg.data[3] = _data[3];
  msg.data[4] = _value.bytes[0];
  msg.data[5] = _value.bytes[1];
  msg.data[6] = _value.bytes[2];
  msg.data[7] = _value.bytes[3];
  return (msg);
}

void get_RS232(){
  char rc[32];
  if (comm_protocols.rs485.available() > 0){
    comm_protocols.rs485.readStringUntil('>').toCharArray(rc,32);
  }
  comm_protocols.rs485.flush();
  if (rc[0]=='L' && rc[5]=='R' && rc[10]=='E' && rc[15]=='S' && rc[20]=='O' && rc[25]=='T'){
    speedL.value = atoi(rc+1);
    memmove(rc, rc+5, sizeof(rc));
    speedR.value = atoi(rc+1);
    memmove(rc, rc+5, sizeof(rc));
    soft_estop.value = atoi(rc+1);
    memmove(rc, rc+5, sizeof(rc));
    cpu_status.value = atoi(rc+1);
    memmove(rc, rc+5, sizeof(rc));
    ctrl_bytes.value = atoi(rc+1);
  }
}

long msg_data_2_long(byte _bytes[]){
  long _long_num = (_bytes[7]<<24) | (_bytes[6]<<16) | (_bytes[5]<<8) | (_bytes[4]);
  return (_long_num);
}

void send_can_msg(mbed::CANMessage msg){
  if (comm_protocols.can.write(msg)){
      ;
  }
  else {
    Serial.println("Transmission Error: ");
    Serial.println(comm_protocols.can.tderror());
    comm_protocols.can.reset();
  }

  if(comm_protocols.can.read(in_msg)){
    if (in_msg.data[1] == 0x04){
      if (in_msg.data[3] == 0x01){
        memmove(encoderL_res.bytes,in_msg.data+4,4);
        }
      else {
        memmove(encoderR_res.bytes,in_msg.data+4,4);
        }
    }
    if (in_msg.data[1] == 0x3A){
      memmove(soc_res.bytes,in_msg.data+4,4);
    }
  }
  delay(10);
}

void print_rs232_data(){
    Serial.print("Left speed: ");
    Serial.print(speedL.value);
    Serial.print("\t|\tRight speed: ");
    Serial.print(speedR.value);
    Serial.print("\t|\tEstop: ");
    Serial.print(soft_estop.value);
    Serial.print("\t|\tStatus: ");
    Serial.print(cpu_status.value);
    Serial.print("\t|\tOutputs: ");
    Serial.println(ctrl_bytes.value);
}

void print_can_data(){
  Serial.print("ID: ");
  Serial.print(in_msg.id);
  Serial.print("\t|\tResponse: ");
  Serial.print(in_msg.data[1],HEX);
  Serial.print("\t|\tLong: ");
  Serial.println(msg_data_2_long(in_msg.data));
}

void send_RS232(char msg[]){
  comm_protocols.rs485.noReceive();
  comm_protocols.rs485.beginTransmission();
  comm_protocols.rs485.println(msg);
  comm_protocols.rs485.endTransmission();
  comm_protocols.rs485.receive();
}

void emergency_state(){
  speedL.value = 0;
  speedR.value = 0;
  state_res.value = 404;
  send_can_msg(create_msg_obj(speedL_comm,speedL));
  send_can_msg(create_msg_obj(speedR_comm,speedR));
  sprintf(msg_msg, "L%iR%iC%iS%i", encoderL_res.value, encoderR_res.value, soc_res.value,state_res.value);
  send_RS232(msg_msg);
  digital_outputs.set(tray_A, LOW);
  digital_outputs.set(tray_B, LOW);
  digital_outputs.set(blade_A, LOW);
  digital_outputs.set(blade_B, LOW);
  while (!digital_inputs.read(estop_pin) || !digital_inputs.read(green_pin)){
    Serial.println("EMERGENCY!!!!!");
    digital_outputs.set(buzzer, HIGH);
    delay(1000);
    digital_outputs.set(buzzer, LOW);
    delay(1000);
  }
}

void chop(int repeats){
  for (int i=0; i<repeats; i++){
    digital_outputs.set(blade_A, LOW);
    digital_outputs.set(blade_B, HIGH);
    delay(300);
    digital_outputs.set(blade_A, HIGH);
    digital_outputs.set(blade_B, LOW);
    delay(600);
  }
  digital_outputs.set(blade_A, LOW);
  digital_outputs.set(blade_B, LOW);
}

void setup() {
  //Setup Outputs
  digital_outputs.setLatch();
  digital_outputs.setAll(0);
  digital_outputs.set(blade_A, LOW);
  digital_outputs.set(blade_B, LOW);
  digital_outputs.set(blade_A, HIGH);
  delay(1000);
  digital_outputs.set(blade_A, LOW);
  
  //Start Serial
  Serial.begin(9600);
  Serial.println("Serial initialization done!");

  //Setup Inputs
  Wire.begin();
  if (!digital_programmables.init()) {
    Serial.println("GPIO expander initialization fail!!");
  }
  if (!digital_inputs.init()) {
    Serial.println("Digital input GPIO expander initialization fail!!");
  }

  //Start RS232
  Serial.println("Starting RS232");
  comm_protocols.rs485Enable(true);
  comm_protocols.rs485ModeRS232(true);
  comm_protocols.rs485.begin(115200);
  comm_protocols.rs485.setTimeout(20);
  comm_protocols.rs485.receive();
  Serial.println("RS232 initialization done!");

  //Wait for the CPU to send the "OK" signal via RS232
  char hs_msg[32] = "L0R0C0S0";
  while (!rs232_ok){
    digital_outputs.set(buzzer, HIGH);
    send_RS232(hs_msg);
    get_RS232();
    //print_rs232_data();
    digital_outputs.set(buzzer, LOW);
    delay(500);
    if (speedL.value == 0 && speedR.value == 0 && soft_estop.value == 0 && cpu_status.value == 0) {
      Serial.println("Hands Shaken");
      rs232_ok = true;
    }
  }

  //Power up the motors controller
  Serial.println("Starting the Motors Controller");
  digital_outputs.set(enbl_motors, HIGH);
  Serial.println("Motors Controller started");
  
  //Wait for the controller to start up
  delay(10000);
  
  //Start the CAN Bus
  Serial.println("Start CAN initialization");
  comm_protocols.enableCAN();
  comm_protocols.can.frequency(canRate);
  Serial.println("CAN initialization done!");

  //Wait for the CAN Bus to send the "OK" signal
  while (!can_ok){
    send_can_msg(create_msg_obj(flags_msg));
    if (in_msg.data[0]==0x4B && msg_data_2_long(in_msg.data)==0){
      can_ok = true;
    }
  }
  Serial.println("CAN Ok");
  Serial.println("Moving into main routine");
}

void loop (){
  static eState LoopState = STATE_START;
  eState NextState = LoopState;
  static bool picking = false;
  static bool rotating_tray = false;
  static long rotation_start = 0;
  
  switch (LoopState) {
  case STATE_START:
  //Initial empty state
      NextState = STATE_READ_RS232;
      break;
  case STATE_READ_RS232:
  //Read and write RS232 data if available
      state_res.value = 1;
      if (comm_protocols.rs485.available()>0){
        get_RS232();
        //print_rs232_data();
        last_mot_cmd_time = millis();
        sprintf(msg_msg, "L%iR%iC%iS%i", encoderL_res.value, encoderR_res.value, soc_res.value,state_res.value);
        send_RS232(msg_msg);
      }
      else if (millis() >= last_mot_cmd_time+motor_command_to){
        speedL.value = 0;
        speedR.value = 0;
      }
      NextState = STATE_CHECK_IO;
      break;
  case STATE_CHECK_IO:
  //Check IOs and Control bytes 
      state_res.value = 2;
      if (!digital_inputs.read(estop_pin) || soft_estop.value!=0)
        emergency_state();
      picking = (ctrl_bytes.value == 1 && millis()>=last_pick+10000) ? true : picking;
      NextState = (picking) ? STATE_DROP_BLADE : STATE_READ_CAN;
      break;
  case STATE_DROP_BLADE:
  //Drop the blade
      state_res.value = 100;
      chop(2);
      NextState = STATE_ROTATE_TRAY;
      break;
  case STATE_ROTATE_TRAY:
  //Rotate the tray until a sensor is detected or 1.2 seconds passed
      rotation_start = (rotating_tray) ? rotation_start : millis();
      digital_outputs.set(tray_A, LOW);
      digital_outputs.set(tray_B, HIGH);
      rotating_tray = true;
      delay(500);
      while (!digital_inputs.read(sensor_pin) && (millis() < rotation_start + 600)){
        ;
      }
      delay(100);
      digital_outputs.set(tray_A, LOW);
      digital_outputs.set(tray_B, LOW);
      state_res.value = 200; 
      rotating_tray = false;
      picking = false;
      last_pick = millis();
      NextState = STATE_READ_CAN;
      break;
  case STATE_READ_CAN:
      send_can_msg(create_msg_obj(speedL_comm,speedL));
      send_can_msg(create_msg_obj(speedR_comm,speedR));
      send_can_msg(create_msg_obj(encoderL_msg));
      send_can_msg(create_msg_obj(encoderR_msg));
      if (millis() >= last_non_crit+non_crit_values_freq){
        if (speedL.value!=0 || speedR.value!=0) {digital_outputs.set(buzzer, HIGH);}
        send_can_msg(create_msg_obj(flags_msg));
        //if (speedL.value>=0 || speedR.value>=0) {digital_outputs.set(buzzer, LOW);}
        send_can_msg(create_msg_obj(voltage_msg));
        send_can_msg(create_msg_obj(soc_msg));
        last_non_crit = millis();
        digital_outputs.set(buzzer, LOW);
      }
      NextState = STATE_START;
      break;
  default:
      NextState = STATE_START;
  }
  LoopState = NextState;
}
