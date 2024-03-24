// switch parts
int switch_pwm_A = 0;
int switch_pwm_B = 0;
bool usePIDCompute = false;
float spd_rate_A = 1.0;
float spd_rate_B = 1.0;
bool heartbeatStopFlag = false;

void movtionPinInit(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  ledcSetup(channel_A, freq, ANALOG_WRITE_BITS);
  ledcAttachPin(PWMA, channel_A);

  ledcSetup(channel_B, freq, ANALOG_WRITE_BITS);
  ledcAttachPin(PWMB, channel_B);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}


void switchEmergencyStop(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}


void switchPortCtrlA(float pwmInputA){
  int pwmIntA = round(pwmInputA * spd_rate_A);
  if(abs(pwmIntA) < 1e-6){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    return;
  }

  if(pwmIntA > 0){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(channel_A, pwmIntA);
  }
  else{
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(channel_A,-pwmIntA);
  }
}


void switchPortCtrlB(float pwmInputB){
  int pwmIntB = round(pwmInputB * spd_rate_B);
  if(abs(pwmIntB) < 1e-6){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    return;
  }

  if(pwmIntB > 0){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(channel_B, pwmIntB);
  }
  else{
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(channel_B,-pwmIntB);
  }
}


void switchCtrl(int pwmIntA, int pwmIntB) {
    switch_pwm_A = pwmIntA;
    switch_pwm_B = pwmIntB;
    switchPortCtrlA(switch_pwm_A);
    switchPortCtrlB(switch_pwm_B);
}


void lightCtrl(int pwmIn) {
  switch_pwm_A = pwmIn;
  switchPortCtrlA(-abs(switch_pwm_A));
}


void setSpdRate(float inputL, float inputR) {
  inputL = abs(inputL);
  if (inputL > 1) {
    inputL = 1;
  }
  inputR = abs(inputR);
  if (inputR > 1) {
    inputR = 1;
  }
  spd_rate_A = inputL;
  spd_rate_B = inputR;
}


void getSpdRate() {
  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = CMD_GET_SPD_RATE;

  jsonInfoHttp["L"] = spd_rate_A;
  jsonInfoHttp["R"] = spd_rate_B;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}



// movtion parts.
// A-left, B-right

ESP32Encoder encoderA;
ESP32Encoder encoderB;

static unsigned long lastTime = 0;
static unsigned long lastLeftSpdTime = 0;
static unsigned long lastRightSpdTime = 0;
int lastEncoderA = 0;
int lastEncoderB = 0;

double speedGetA;
double speedGetB;

double plusesRate = 3.14159265359 * WHEEL_D / ONE_CIRCLE_PLUSES;


void initEncoders() {
  // if(SET_MOTOR_DIR){
  //   encoderA.attachHalfQuad(AENCB, AENCA);
  //   encoderB.attachHalfQuad(BENCB, BENCA);
  // }else{
  encoderA.attachHalfQuad(AENCA, AENCB);
  encoderB.attachHalfQuad(BENCA, BENCB);
  // }
  encoderA.setCount(0);
  encoderB.setCount(0);
}

void getWheelSpeed() {
  unsigned long currentTime = micros();
  long encoderPulsesA = encoderA.getCount();
  long encoderPulsesB = encoderB.getCount();

  if (!SET_MOTOR_DIR) {
    speedGetA = (plusesRate * (encoderPulsesA - lastEncoderA)) / ((double)(currentTime - lastTime) / 1000000);
    speedGetB = (plusesRate * (encoderPulsesB - lastEncoderB)) / ((double)(currentTime - lastTime) / 1000000);
  } else {
    speedGetA = (plusesRate * (lastEncoderA - encoderPulsesA)) / ((double)(currentTime - lastTime) / 1000000);
    speedGetB = (plusesRate * (lastEncoderB - encoderPulsesB)) / ((double)(currentTime - lastTime) / 1000000);
  }
  lastEncoderA = encoderPulsesA;
  lastEncoderB = encoderPulsesB;
  lastTime = currentTime;
}

void getLeftSpeed() {
  unsigned long currentTime = micros();
  long encoderPulsesA = encoderA.getCount();
  if (!SET_MOTOR_DIR) {
    speedGetA = (plusesRate * (encoderPulsesA - lastEncoderA)) / ((double)(currentTime - lastLeftSpdTime) / 1000000);
  } else {
    speedGetA = (plusesRate * (lastEncoderA - encoderPulsesA)) / ((double)(currentTime - lastLeftSpdTime) / 1000000);
  }
  lastEncoderA = encoderPulsesA;
  lastLeftSpdTime = currentTime;
}

void getRightSpeed() {
  unsigned long currentTime = micros();
  long encoderPulsesB = encoderB.getCount();
  if (!SET_MOTOR_DIR) {
    speedGetB = (plusesRate * (encoderPulsesB - lastEncoderB)) / ((double)(currentTime - lastRightSpdTime) / 1000000);
  } else {
    speedGetB = (plusesRate * (lastEncoderB - encoderPulsesB)) / ((double)(currentTime - lastRightSpdTime) / 1000000);
  }
  lastEncoderB = encoderPulsesB;
  lastRightSpdTime = currentTime;
}





// --- PID Controller ---

PID_v2 pidA(__kp, __ki, __kd, PID::Direct);
PID_v2 pidB(__kp, __ki, __kd, PID::Direct);

double outputA = 0;
double outputB = 0;
double setpointA = 0;
double setpointB = 0;

int setpoint_interval = 200;
unsigned long setpoint_cmd_recv = millis();
unsigned long setpoint_last_time = millis();
float setpointA_buffer;
float setpointB_buffer;
float setpointA_last;
float setpointB_last;
float change_offset = 0.005;
bool new_setpoint_flag = false;

void pidControllerInit() {
  pidA.Start(speedGetA,
             outputA,
             setpointA);
  pidA.SetOutputLimits(-255, 255);
  pidA.SetMode(PID::Automatic);

  pidB.Start(speedGetB,
             outputB,
             setpointB);
  pidB.SetOutputLimits(-255, 255);
  pidB.SetMode(PID::Automatic);
}

void leftCtrl(float pwmInputA){
  int pwmIntA = round(pwmInputA);
  if (mainType != 3) {
    speedGetA = pwmIntA;
  }
  if(SET_MOTOR_DIR){
    if(pwmIntA < 0){
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      ledcWrite(channel_A, abs(pwmIntA));
    }
    else{
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      ledcWrite(channel_A, abs(pwmIntA));
    }
  }else{
    if(pwmIntA < 0){
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      ledcWrite(channel_A, abs(pwmIntA));
    }
    else{
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      ledcWrite(channel_A, abs(pwmIntA));
    }
  }
}

void rightCtrl(float pwmInputB){
  int pwmIntB = round(pwmInputB);
  if (mainType != 3) {
    speedGetB = pwmIntB;
  }
  if(SET_MOTOR_DIR){
    if(pwmIntB < 0){
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      ledcWrite(channel_B, abs(pwmIntB));
    }
    else{
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      ledcWrite(channel_B, abs(pwmIntB));
    }
  }else{
    if(pwmIntB < 0){
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      ledcWrite(channel_B, abs(pwmIntB));
    }
    else{
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      ledcWrite(channel_B, abs(pwmIntB));
    }
  }
}

void setGoalSpeed(float inputLeft, float inputRight) {
  // setpoint_cmd_recv = millis();
  if (mainType == 3) {
    usePIDCompute = true;

    if(inputLeft < -2.0 || inputLeft > 2.0){
      return;
    }

    if(inputRight < -2.0 || inputRight > 2.0){
      return;
    }
    
    setpointA = inputLeft*spd_rate_A;
    setpointB = inputRight*spd_rate_B;

    if (setpointA != setpointA_buffer) {
      pidA.Setpoint(setpointA);
      setpointA_buffer = inputLeft;
    }
    
    if (setpointB != setpointB_buffer) {
      pidB.Setpoint(setpointB);
      setpointB_buffer = inputRight;
    }
  } else {
    usePIDCompute = false;
    leftCtrl(inputLeft * 512 * spd_rate_A);
    rightCtrl(inputRight * 512 * spd_rate_B);
  }
}

void pidControllerCompute() {
  if (!usePIDCompute) {
    return;
  }

  outputA = pidA.Run(speedGetA);
  if (abs(outputA)<THRESHOLD_PWM) {
    outputA = 0;
  }
  if (setpointA == 0 && speedGetA == 0) {
    outputA = 0;
  }
  leftCtrl(outputA);

  outputB = pidB.Run(speedGetB);
  if (abs(outputB)<THRESHOLD_PWM) {
    outputB = 0;
  }
  if (setpointB == 0 && speedGetB == 0) {
    outputB = 0;
  }
  rightCtrl(outputB);
}

void LeftPidControllerCompute() {
  if (!usePIDCompute) {
    return;
  }

  outputA = pidA.Run(speedGetA);
  if (abs(outputA)<THRESHOLD_PWM) {
    outputA = 0;
  }
  if (setpointA == 0 && speedGetA == 0) {
    outputA = 0;
  }
  leftCtrl(outputA);
}

void RightPidControllerCompute() {
  if (!usePIDCompute) {
    return;
  }

  outputB = pidB.Run(speedGetB);
  if (abs(outputB)<THRESHOLD_PWM) {
    outputB = 0;
  }
  if (setpointB == 0 && speedGetB == 0) {
    outputB = 0;
  }
  rightCtrl(outputB);
}

void setPID(float inputP, float inputI, float inputD, float inputLimits) {
  __kp = inputP;
  __ki = inputI;
  __kd = inputD;
  windup_limits = inputLimits;
  pidA.SetTunings(__kp, __ki, __kd);
  pidB.SetTunings(__kp, __ki, __kd);
}

void rosCtrl(float rosX, float rosZ) {
  setpointA = rosX - (rosZ * TRACK_WIDTH / 2.0);
  setpointB = rosX + (rosZ * TRACK_WIDTH / 2.0);
  setGoalSpeed(setpointA, setpointB);
}

void heartBeatCtrl() {
  if (currentTimeMillis - lastCmdRecvTime > HEART_BEAT_DELAY) {
    if (!heartbeatStopFlag) {
      heartbeatStopFlag = true;
      setGoalSpeed(0, 0);
      // leftCtrl(0);
      // rightCtrl(0);
    }
  }
}

void changeHeartBeatDelay(int inputCmd) {
  HEART_BEAT_DELAY = inputCmd;
}

void mm_settings(byte inputMain, byte inputModule) {
  mainType = inputMain;
  moduleType = inputModule;

  if (mainType == 1) {
    WHEEL_D = 0.0800;
    ONE_CIRCLE_PLUSES = 2100;
    TRACK_WIDTH = 0.125;
    SET_MOTOR_DIR = false; // checked
    usePIDCompute = false;
  } else if (mainType == 2) {
    WHEEL_D = 0.0800;
    ONE_CIRCLE_PLUSES = 1650;
    TRACK_WIDTH = 0.172;
    SET_MOTOR_DIR = true; // checked
    usePIDCompute = false;
  } else if (mainType == 3) {
    WHEEL_D = 0.0523;
    ONE_CIRCLE_PLUSES = 1092;
    TRACK_WIDTH = 0.141;
    SET_MOTOR_DIR = true; // checked
    usePIDCompute = true;
  }
  plusesRate = 3.14159265359 * WHEEL_D / ONE_CIRCLE_PLUSES;
  // initEncoders();

  if (mainType == 1) {
    screenLine_2 = "RaspRover";
  } else if (mainType == 2) {
    screenLine_2 = "UGV02";
  } else if (mainType == 3) {
    screenLine_2 = "UGV01";
  } 

  if (moduleType == 0) {
    screenLine_2 += " Null";
  } else if (moduleType == 1) {
    screenLine_2 += " Arm";
  } else if (moduleType == 2) {
    screenLine_2 += " PT";
  } 
}