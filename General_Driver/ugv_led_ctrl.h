void led_pin_init(){
  pinMode(IO4_PIN, OUTPUT);
  pinMode(IO5_PIN, OUTPUT);

  ledcSetup(IO4_CH, FREQ, ANALOG_WRITE_BITS);
  ledcSetup(IO5_CH, FREQ, ANALOG_WRITE_BITS);

  ledcAttachPin(IO4_PIN, IO4_CH);
  ledcAttachPin(IO5_PIN, IO5_CH);
}

void led_pwm_ctrl(int io4Input, int io5Input) {
  ledcWrite(IO4_CH, constrain(io4Input, 0, 255));
  ledcWrite(IO5_CH, constrain(io5Input, 0, 255));
}