#include <MyActuator.h>
#include <ClearpathSD.h>

MyActuator canbus = MyActuator();

const int MOTORCOUNT = 6;

// define pins
const int HLFB = A3;
const int inputB = 10;
const int inputA = A2;
const int enable = A0;
const int prox = 7;

ClearpathSD zmotor = ClearpathSD(HLFB, inputA, inputB, enable, prox);

// Canbus motors CANIDs
int16_t RMDX8 = 0x144;
int16_t RMDX6 = 0x143;
int16_t xg1 = 0x142;
int16_t xg2 = 0x146;
int16_t xg3 = 0x147;

// if RMDL is within 1 rotation of zero position, it will be correct
int32_t xg1zero = 0x0;
int32_t xg2zero = 0x0;
int32_t xg3zero = 0x0;
int32_t x8zero = 0x0;
int32_t x6zero = 0x0;

// multitasking 
long int listenInterval = 20;
long int motionInterval = 20;
long int listenTime = 0;
long int motionTime = 0;

// zmotor direction -1(down) or 1(up)
int zdirection = 1;
long int zpos = 0;
long int ztarget;

// canbus motor positions
int32_t x8pos = 0;
int32_t x6pos = 0;
int32_t xg1pos = 0;
int32_t xg2pos = 0;
int32_t xg3pos = 0;

int16_t x8spd = 0;
int16_t x6spd = 0;
int16_t x4spd = 0;
int16_t xg1spd = 0;
int16_t xg2spd = 0;
int16_t xg3spd = 0;

int32_t x8target = 0;
int32_t x6target = 0;
int32_t x4target = 0;
int32_t xg1target = 0;
int32_t xg2target = 0;
int32_t xg3target = 0;

bool x8inRange = false;
bool x6inRange = false;
bool x4inRange = false;
bool xlinRange = false;
bool zinRange = false;
bool moving = false;

int errorStatus = 0x0;

// Operation states
int runPickerState = 1;

// FIFO queue
const int qsize = 100;
const int esize = 10;
byte queue[qsize][esize];
int front = 0;
int rear = -1;
int itemCount = 0;
byte element[10];

// zmotor interrupt
// when calling absPos, zdirection must be set
// when calling absPos, ztarget must also be set
// when homing, set desiredzpos
ISR(TIMER1_OVF_vect){
  zpos = zpos + zdirection;
  if (zpos == ztarget){
    zmotor.stop();
  }
}

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  canbus.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  // Tasks
  // Read 64 byte serial buffer
  // Filter by instructions
  // if Move command, append to move buffer
  // if Canbus command, execute immediately
  // Check speed and positions of all motors
  // execute all move commands when ready (motors in range and not moving)
  // listen Serial every serial interval
  trackMotion();
  listenSerial();

}

// start move commands
void beginMotion(){
  // check if queue is empty

  if (itemCount != 0 && runPickerState == 2){
    
    // execute 6 move commands from move queue
    for (int i = 0; i < 6; i++){
      // get move command from move queue
      dequeue();

      if (element != NULL){
        int16_t motorID = 0x0;
        motorID = motorID | (int16_t)element[0];
        motorID = motorID | ((int16_t)element[1] << 8); 

        // Check if Teknic move or CANBUS move
        if (motorID == 0x140){
          if (element[2] == 0xA4){
            // Absolute Move
            // Move Teknic Motor
            // convert to desired z pos
            int32_t angleControl = 0x0;
            angleControl = angleControl | ((int32_t)element[6]);
            angleControl = angleControl | ((int32_t)element[7] << 8);
            angleControl = angleControl | ((int32_t)element[8] << 16);
            angleControl = angleControl | ((int32_t)element[9] << 24);
            
            int16_t speedControl = 0x0;
            speedControl = speedControl | ((int16_t)element[4]);
            speedControl = speedControl | ((int16_t)element[5] << 8);
            
            int16_t spdd = (double)speedControl / 360.0 * 60.0;
            zmotor.setSpeed(spdd);
            
            ztarget = (int32_t)((double)angleControl / 36000 / 19.113125 * 122324);
            zdirection = zmotor.setAbsPos(ztarget);
          } else if (element[2] == 0xA8){
            // Incremental Move
            // Move Teknic Motor
            // convert to desired z pos
            int32_t angleControl = 0x0;
            angleControl = angleControl | ((int32_t)element[6]);
            angleControl = angleControl | ((int32_t)element[7] << 8);
            angleControl = angleControl | ((int32_t)element[8] << 16);
            angleControl = angleControl | ((int32_t)element[9] << 24);
            
            int16_t speedControl = 0x0;
            speedControl = speedControl | ((int16_t)element[4]);
            speedControl = speedControl | ((int16_t)element[5] << 8);
            
            int16_t spdd = (double)speedControl / 360.0 * 60.0;
            zmotor.setSpeed(spdd);
            
            ztarget = zpos + (int32_t)((double)angleControl / 36000 / 19.113125 * 122324);
            zdirection = zmotor.setAbsPos(ztarget);
          }
        } else { 
          // Move canbus Motor
          // filter by abs or incremental move
          if (element[2] == 0xA4){
            // Absolute Move
            int32_t angleControl = 0x0;
            angleControl = angleControl | ((int32_t)element[6]);
            angleControl = angleControl | ((int32_t)element[7] << 8);
            angleControl = angleControl | ((int32_t)element[8] << 16);
            angleControl = angleControl | ((int32_t)element[9] << 24);
            
            int16_t maxSpeed = 0x0;
            maxSpeed = maxSpeed | ((int16_t)element[4]);
            maxSpeed = maxSpeed | ((int16_t)element[5] << 8);

            if (motorID == RMDX8) {
              x8target = angleControl + x8zero;
            } else if (motorID == RMDX6){
              x6target = angleControl + x6zero;
            } else if (motorID == xg1){
              xg1target = angleControl + xg1zero;
            }

            int32_t angleAdjusted = 0x0;
            if (motorID == xg1){
              angleAdjusted = angleControl + xg1zero;
              canbus.setAbsPos(motorID, angleAdjusted, maxSpeed);
              canbus.checkReceived(0);
            } else if (motorID == xg2){
              angleAdjusted = angleControl + xg2zero;
              canbus.setAbsPos(motorID, angleAdjusted, maxSpeed);
              canbus.checkReceived(0);
            } else if (motorID == xg3){
              angleAdjusted = angleControl + xg3zero;
              canbus.setAbsPos(motorID, angleAdjusted, maxSpeed);
              canbus.checkReceived(0);
            } else if (motorID == RMDX8){
              angleAdjusted = angleControl + x8zero;
              canbus.setAbsPos(motorID, angleAdjusted, maxSpeed);
              canbus.checkReceived(0);
            } else if (motorID == RMDX6){
              angleAdjusted = angleControl + x6zero;
              canbus.setAbsPos(motorID, angleAdjusted, maxSpeed);
              canbus.checkReceived(0);
            }
            
          } else if (element[2] == 0xA8) {
            // incremental Move
            int32_t angleControl = 0x0;
            angleControl = angleControl | ((int32_t)element[6]);
            angleControl = angleControl | ((int32_t)element[7] << 8);
            angleControl = angleControl | ((int32_t)element[8] << 16);
            angleControl = angleControl | ((int32_t)element[9] << 24);
            
            int16_t maxSpeed = 0x0;
            maxSpeed = maxSpeed | ((int16_t)element[4]);
            maxSpeed = maxSpeed | ((int16_t)element[5] << 8);

            if (motorID == RMDX8) {
              x8target = x8pos - x8zero + angleControl;
            } else if (motorID == RMDX6){
              x6target = x6pos - x6zero + angleControl;
            } else if (motorID == xg1){
              xg1target = xg1pos - xg1zero + angleControl;
            }
            
            canbus.setIncrementalPos(motorID, angleControl, maxSpeed);
            canbus.checkReceived(0);
          }
        }
      }
    }
  } else {
    // if queue is empty there all moves are complete
    runPickerState = 1; 
  }
}

// track position of all motors
// if motion is complete, start next moves
// if all moves are complete, send move complete flag
void trackMotion(){
  // reset pos
  x8pos = 0;
  x6pos = 0;
  xg1pos = 0;
  xg2pos = 0;
  xg3pos = 0;

  x8inRange = false;
  x6inRange = false;
  x4inRange = false;
  xlinRange = false;

  int64_t canReplyRMDX8 = 0x0;

//  Serial.println("start");
  canbus.getAbsAngle(RMDX8);
  canReplyRMDX8 = canbus.checkReceived(0);
  x8pos = x8pos | (int32_t)((canReplyRMDX8 & 0xFF000000) >> 24);
  x8pos = x8pos | (int32_t)((canReplyRMDX8 & 0x00FF0000) >> 8);
  x8pos = x8pos | (int32_t)((canReplyRMDX8 & 0x0000FF00) << 8);
  x8pos = x8pos | (int32_t)((canReplyRMDX8 & 0x000000FF) << 24);

  int64_t canReplyRMDX6 = 0x0;
  
  canbus.getAbsAngle(RMDX6);
  canReplyRMDX6 = canbus.checkReceived(0);
  x6pos = x6pos | (int32_t)((canReplyRMDX6 & 0xFF000000) >> 24);
  x6pos = x6pos | (int32_t)((canReplyRMDX6 & 0x00FF0000) >> 8);
  x6pos = x6pos | (int32_t)((canReplyRMDX6 & 0x0000FF00) << 8);
  x6pos = x6pos | (int32_t)((canReplyRMDX6 & 0x000000FF) << 24);

  int64_t canReplyxg1 = 0x0;
  
  // get Abs angle 
  // encoder only rotates 1 full revolution
  // reply: 142 92 xx xx FF FF FF FF FF
  canbus.getAbsAngle(xg1);
  canReplyxg1 = canbus.checkReceived(0);
  xg1pos = xg1pos | (int32_t)((canReplyxg1 & 0x00FF000000000000) >> 48);
  xg1pos = xg1pos | (int32_t)((canReplyxg1 & 0x0000FF0000000000) >> 32);
  xg1pos = xg1pos | (int32_t)((canReplyxg1 & 0x000000FF00000000) >> 16);
  xg1pos = xg1pos | (int32_t)(canReplyxg1 & 0x00000000FF000000);
  
  
  int64_t canReplyxg2 = 0x0;

  canbus.getAbsAngle(xg2);
  canReplyxg2 = canbus.checkReceived(0);
  xg2pos = xg2pos | (int32_t)((canReplyxg2 & 0x00FF000000000000) >> 48);
  xg2pos = xg2pos | (int32_t)((canReplyxg2 & 0x0000FF0000000000) >> 32);
  xg2pos = xg2pos | (int32_t)((canReplyxg2 & 0x000000FF00000000) >> 16);
  xg2pos = xg2pos | (int32_t)(canReplyxg2 & 0x00000000FF000000);
  
  
  int64_t canReplyxg3 = 0x0;
   
  canbus.getAbsAngle(xg3);
  canReplyxg3 = canbus.checkReceived(0);
  xg3pos = xg3pos | (int32_t)((canReplyxg3 & 0x00FF000000000000) >> 48);
  xg3pos = xg3pos | (int32_t)((canReplyxg3 & 0x0000FF0000000000) >> 32);
  xg3pos = xg3pos | (int32_t)((canReplyxg3 & 0x000000FF00000000) >> 16);
  xg3pos = xg3pos | (int32_t)(canReplyxg3 & 0x00000000FF000000);
  
//  Serial.println("end");
  // reset speeds
//  x8spd = 0;
//  x6spd = 0;
//  x4spd = 0;
//  xg1spd = 0;
//  
  // get speeds of all motors
//  canbus.getMotorStatus2(RMDX8);
//  reply = canbus.checkReceived(0);
//  x8spd = x8spd | (int16_t)(reply >> 24);
//  x8spd = x8spd | ((int16_t)(reply >> 8) & 0xFF00);
//
//  canbus.getMotorStatus2(RMDX6);
//  reply = canbus.checkReceived(0);
//  x6spd = x6spd | (int16_t)(reply >> 24);
//  x6spd = x6spd | ((int16_t)(reply >> 8) & 0xFF00);

//  canbus.getMotorStatus2(RMDX4);
//  reply = canbus.checkReceived(0);
//  x4spd = x4spd | (int16_t)(reply >> 24);
//  x4spd = x4spd | ((int16_t)(reply >> 8) & 0xFF00);

//  canbus.getMotorStatus2(xg1);
//  reply = canbus.checkReceived(0);
//  xg1spd = xg1spd | (int16_t)(reply >> 24);
//  xg1spd = xg1spd | ((int16_t)(reply >> 8) & 0xFF00);
//  Serial.print(" xg1spd: ");
//  Serial.println(xg1spd);

  int32_t range = 10;

  // check if motors are in range of position
  if (x8pos >= x8target - range && x8pos <= x8target + range){
    x8inRange = true;
  } else {
    x8inRange = false;
  }

   // check if motors are in range of position
  if (x6pos >= x6target - range && x6pos <= x6target + range){
    x6inRange = true; 
  } else {
    x6inRange = false;
  }

  
//   // check if motors are in range of position
//  if (xg1pos >= xg1target - range && xg1pos <= xg1target + range){
//    xlinRange = true;
//  } else {
//    xlinRange = false;
//  }

  zinRange = (zpos == ztarget);
  bool inRange = x8inRange && x6inRange && zinRange;
    
  // start next set of moves if the previous ones have been completed
  
  if (inRange && runPickerState == 2 ){
    beginMotion();
  }
}

// Listen Serial
// Read 64 bytes from serial buffer 
// extract motorID and commandID
// filter by move command and non move command
// send non move commands
// store move commands
void listenSerial(){
  // if serial buffer contains a CANBUS msg
  int msgs = 0;
  byte sbuf[10];
  byte mbuf[(MOTORCOUNT - 1) * 10];
  
  if (Serial.available() > 9){

    msgs = Serial.available()/10;
    
    for (int i = 0; i < 10; i++){
      sbuf[i] = Serial.read();
    }

    // for each available message, store it and send to CAN
    // can support up to 6 serial messages per loop
    // Serial message format = 10 bytes 
    // ID followed by 8 bytes of can message
    // Read all available bytes in serial buffer    
//    for (int j = 0; j < 64; j++){
//      sbuf[j] = Serial.read();
//    }

    // check if 60 bytes is a move command
    // 60 bytes MUST BE 6 move commands
    if (sbuf[2] == 0xA4 || sbuf[2] == 0xA8){
      // baud rate = 115200
      // 14400 bytes per 1000ms
      // 1000ms / 14400 bytes * 60 = 4.166ms transmit time
      
      long int startWait = millis();

      while (Serial.available() < (MOTORCOUNT - 1)*10){
        long int waitTime = millis();        
        // if 10ms have elapsed break loop
        if (waitTime - startWait > 100){
          break;
        }
      }
      
      for (int i = 0; i < (MOTORCOUNT - 1)*10; i++){
        mbuf[i] = Serial.read();
      }
      
      byte mmsg[10]= {0,0,0,0,0,0,0,0,0,0};
      
      mmsg[0] = sbuf[0];
      mmsg[1] = sbuf[1];
      mmsg[2] = sbuf[2];
      mmsg[3] = sbuf[3];
      mmsg[4] = sbuf[4];
      mmsg[5] = sbuf[5];
      mmsg[6] = sbuf[6];
      mmsg[7] = sbuf[7];
      mmsg[8] = sbuf[8];
      mmsg[9] = sbuf[9];

      enqueue(mmsg);
      
      for(int i = 0; i < 5; i++){
        mmsg[0] = mbuf[10*i];
        mmsg[1] = mbuf[10*i + 1];
        mmsg[2] = mbuf[10*i + 2];
        mmsg[3] = mbuf[10*i + 3];
        mmsg[4] = mbuf[10*i + 4];
        mmsg[5] = mbuf[10*i + 5];
        mmsg[6] = mbuf[10*i + 6];
        mmsg[7] = mbuf[10*i + 7];
        mmsg[8] = mbuf[10*i + 8];
        mmsg[9] = mbuf[10*i + 9];

        enqueue(mmsg);
      }
    }else{
      // if not move command
      // read 10 bytes at a time
      for (int i = 0; i < msgs; i++){
        byte cmsg[8] = {0,0,0,0,0,0,0,0};
        int16_t motorID = 0x0;
        
        // extract motor ID
        motorID = motorID | (int16_t)sbuf[10*i];
        motorID = motorID | ((int16_t)sbuf[10*i + 1] << 8);
        
        // pkg can message into 8 byte array
        cmsg[0] = sbuf[10*i + 2];
        cmsg[1] = sbuf[10*i + 3];
        cmsg[2] = sbuf[10*i + 4];
        cmsg[3] = sbuf[10*i + 5];
        cmsg[4] = sbuf[10*i + 6];
        cmsg[5] = sbuf[10*i + 7];
        cmsg[6] = sbuf[10*i + 8];
        cmsg[7] = sbuf[10*i + 9];  

        translateAndSend(motorID, cmsg);
      }
    }
  }
}

// get current position of all motors and save as new zero position
void homeMotors(){
  trackMotion();
  if (runPickerState == 1){
    x8zero = x8pos;
    x6zero = x6pos;
    xg1zero = xg1pos;
    xg2zero = xg2pos;
    xg3zero = xg3pos;
  
    zmotor.home();
    zpos = 0;
  }
}

void translateAndSend(int16_t motorID, byte cmsg[]){

  byte commandID = cmsg[0];
  
  // filter by command first
  if (commandID == 0xD1){
    // start move command
    if (itemCount != 0){
      zmotor.engage();
      runPickerState = 2;
      beginMotion();
    }
  } else if (commandID == 0xD2) {
    // clear move buffer
    clearQueue();
    
  } else if (commandID == 0x9A){
    // poll Motors
    pollMotors();
  }else if (commandID == 0x63){
    // home
    homeMotors();    
  }else if (commandID == 0x8C){
    // engage
    // set picker state to engaged
    zmotor.engage();
    
  }else if (commandID == 0x8B){
    // disengage
    // set picker state to idle
    
    zmotor.disengage();
    ztarget = zpos;
    
    // disengage all canbus motors
    canbus.shutdown(RMDX8);
    canbus.checkReceived(0);
    canbus.shutdown(RMDX6);
    canbus.checkReceived(0);
    canbus.shutdown(xg1);
    canbus.checkReceived(0);
//    canbus.shutdown(xg2);
//    canbus.checkReceived(0);
    canbus.shutdown(xg3);
    canbus.checkReceived(0);
    
  }else if (commandID == 0x81){
    // stop
    canbus.stop(RMDX8);
    canbus.checkReceived(0);
    canbus.stop(RMDX6);
    canbus.checkReceived(0);
    canbus.stop(xg1);
    canbus.checkReceived(0);
//    canbus.stop(xg2);
//    canbus.checkReceived(0);
    canbus.stop(xg3);
    canbus.checkReceived(0);
    
    zmotor.stop();
    ztarget = zpos;
    // set picker state to engaged
    
  }else if (commandID == 0x80){
    // shutdown
    canbus.shutdown(RMDX8);
    canbus.checkReceived(0);
    canbus.shutdown(RMDX6);
    canbus.checkReceived(0);
    canbus.shutdown(xg1);
    canbus.checkReceived(0);
//    canbus.shutdown(xg2);
//    canbus.checkReceived(0);
    canbus.shutdown(xg3);
    canbus.checkReceived(0);

    zmotor.disengage();
    ztarget = zpos;
    // set picker state to idle
    
  }else if (commandID == 0x78){
    // brake lock
    canbus.brakeLock(RMDX8);
    canbus.checkReceived(0);
    canbus.brakeLock(RMDX6);
    canbus.checkReceived(0);
    
  }else if (commandID == 0x77){
    // brake release
    canbus.brakeRelease(RMDX8);
    canbus.checkReceived(0);
    canbus.brakeRelease(RMDX6);
    canbus.checkReceived(0);
    
  }else if (commandID == 0x76){
    // reset
    canbus.shutdown(RMDX8);
    canbus.checkReceived(0);
    canbus.shutdown(RMDX6);
    canbus.checkReceived(0);
    canbus.shutdown(xg1);
    canbus.checkReceived(0);
//    canbus.shutdown(xg2);
//    canbus.checkReceived(0);
    canbus.shutdown(xg3);
    canbus.checkReceived(0);

    zmotor.disengage();
    ztarget = zpos;
    // set picker state to idle
  }
}

// getMotorStatus1 and getMotorStatus2 for all motors
// 16 BYTES PER MOTOR
// get position and HLFB of teknic motor
// Write all 80 bytes to serial 
void pollMotors(){
  
  int16_t motorIDs[] = {0x142, 0x143, 0x144, 0x146, 0x147};
  int sz = 5;
  byte reply[78];

  for (int i = 0; i < 78; i++){
    reply[i] = 0;
  }

  // for all motors
  for (int i = 0; i < sz; i++){

    int64_t status1 = 0;
    int64_t status2 = 0;
    int64_t posmsg = 0;
  
    // get motor status 1
    canbus.getMotorStatus1(motorIDs[i]);
    status1 = canbus.checkReceived(0);

    // special case for xg1, xg2, xg3
    if (motorIDs[i] == 0x142 || motorIDs[i] == 0x146 || motorIDs[i] == 0x147){
      // package relevant bytes
      reply[13*i] = (byte)(motorIDs[i]);
      reply[13*i + 1] = (byte)(motorIDs[i] >> 8);
      reply[13*i + 2] = 0x01;
      reply[13*i + 3] = (byte)(status1>>8);
      reply[13*i + 4] = (byte)(status1);
    } else {
      // package relevant bytes
      reply[13*i] = (byte)(motorIDs[i]);
      reply[13*i + 1] = (byte)(motorIDs[i] >> 8);
      reply[13*i + 2] = (byte)(status1>>32);
      reply[13*i + 3] = (byte)(status1>>8);
      reply[13*i + 4] = (byte)(status1);
    }
    
    // get motor status 2
    canbus.getMotorStatus2(motorIDs[i]);
    status2 = canbus.checkReceived(0);

    // package relevant bytes
    reply[13*i + 5] = (byte)(status2 >> 40);
    reply[13*i + 6] = (byte)(status2 >> 32);
    reply[13*i + 7] = (byte)(status2 >> 24);
    reply[13*i + 8] = (byte)(status2 >> 16);

    int32_t p = 0x0;
    if (motorIDs[i] == RMDX8){
      p = x8pos - x8zero;
    } else if (motorIDs[i] == RMDX6){
      p = x6pos - x6zero;
    } else if (motorIDs[i] == xg1){
      p = xg1pos - xg1zero;
    } else if (motorIDs[i] == xg2){
      p = xg2pos - xg2zero;
    } else if (motorIDs[i] == xg3){
      p = xg3pos - xg3zero;
    }
    
    reply[13*i + 9] = (byte)(p);
    reply[13*i + 10] = (byte)(p >> 8);
    reply[13*i + 11] = (byte)(p >> 16);
    reply[13*i + 12] = (byte)(p >> 24);
  
  }

  // get information from teknic motor
  reply[65] = 0x40;
  reply[66] = 0x01;
  reply[67] = 0x0;
  reply[68] = 0x0;
  reply[69] = 0x0;
  reply[70] = 0x0;
  reply[71] = 0x0;
  reply[72] = 0x0;
  reply[73] = 0x0;
  
  double zbuf = (double)zpos / 6400.0 * 36000.0;
  int32_t zposcdeg = (int32_t)zbuf;
  
  reply[74] = (byte)zposcdeg ;
  reply[75] = (byte)(zposcdeg >> 8);
  reply[76] = (byte)(zposcdeg >> 16);
  reply[77] = (byte)(zposcdeg >> 24);

  Serial.write(reply, 78);

}

void enqueue(byte element[esize]) {
  if(itemCount < qsize) {
    rear = (rear + 1) % qsize;
    for (int i = 0; i < esize; i++) {
      queue[rear][i] = element[i];
    }
    itemCount++;
  }
}

bool dequeue() {
  if (itemCount > 0) {
    for (int i = 0; i < esize; i++) {
      element[i] = queue[front][i];
    }
    
    front = (front + 1) % qsize;
    itemCount--;
    return true;
  } else {
    return false;
  }
}

bool isQueueFull() {
  return itemCount == qsize;
}

bool isQueueEmpty() {
  return itemCount == 0;
}

void clearQueue() {
  itemCount = 0;
  front = 0;
  rear = -1;
}

void printQueue() {
  int i = front;
  int count = 0;
  while (count < itemCount) {
    Serial.print("Element ");
    Serial.print(count);
    Serial.print(": ");
    for (int j = 0; j < esize; j++) {
      Serial.print(queue[i][j], HEX);
      Serial.print(" ");
    }
    Serial.println();
    i = (i + 1) % qsize;
    count++;
  }
}

//void runPickerFSM(){
//  long int currentTime = millis();
//  
//  switch (runPickerState){
//    case 0: // Idle State
//    
//      // listen Serial every serial interval
//      if (currentTime - listenTime >= listenInterval){
//        listenTime = currentTime;
//        listenSerial();
//      }
//      
//      break;
//    case 1: // Engaged State
//    
//      // listen Serial every serial interval
//      if (currentTime - listenTime >= listenInterval){
//        listenTime = currentTime;
//        listenSerial();
//      }
//      // get position of all motors
//      if (currentTime - motionTime >= motionInterval){
//        motionTime = currentTime;
//        trackMotion();
//      }
//      
//      break;
//    case 2: // Moving State
//    
//      // listen Serial every serial interval
//      if (currentTime - listenTime >= listenInterval){
//        listenTime = currentTime;
//        listenSerial();
//      }
//      // get position of all motors
//      if (currentTime - motionTime >= motionInterval){
//        motionTime = currentTime;
//        trackMotion();
//      }
//      
//      break;
//  }
//}
