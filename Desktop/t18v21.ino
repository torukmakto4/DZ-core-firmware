 ////DZ Customs
 ////Project T18/T19 Firmware
 ////2017-07-10
 ////2017-08-31
 
 ////This targets:
 ////Arduino Pro Mini, 5V/16MHz.
 ////T18 Backplane V1.
 ////ESCs running SimonK.
 ////DRV8825 stepper driver i.e. Pololu/3d printer style carrier board.
 ////NEMA 17 200s/rev motor with LOW LOW LOW inductance and resistance to be able to go fast
 ////on 3S bus voltage. i.e. StepperOnline 17HS16-2004S1 1.1ohm, 2.6mH
 ////scotch yoke bolt drivetrain, 1 shot per rev.
 ////SPST bolt limit switch.
 ////SPDT trigger switch.
 
 ////Features:
 ////Intelligent single-trigger blaster control.
 ////Stepper motor bolt drive with true linear velocity ramps and position error auto-recovery.
 
 ////There is NOT:
 ////Selective fire. That's mostly for people with shitty trigger logic.
 ////It may be added later using an unused pin.
 ////Flywheel speed adjustment. Subcriticality is dumb.
 ////(Software is not a robust safety compliance measure anyway. Change flywheels to change velocity.)
 ////ROF adjustment. The max this sort of drive can do is an excellent general-purpose ROF.
 ////(There isn't a reason to reduce it.)
 ////Really any user settings or controls except the trigger. You turn it on, you shoot it, and you turn it off.
 
 ////TBD: make fire() aware of the limit switch and reset its position index OTF on switch low
 ////     tune all delays and settings over chrono
 ////     add power-on selftest for fly motors to verify controller operation and free rotation without firing
 ////     multi-stage flywheel startup delay
 ////     turn bolt current off earlier after going idle to save more power

 
 ////This uses the per-cycle lightweight calculation of stepper commutation period described in
 ////http://hwml.com/LeibRamp.pdf
 
 ////     ..user torukmakto4 (PCI 970000591, CT-2406)
 ////     DZ Customs Mo'ara
 ////
 ////     DZ Field Office Terranova/Lakeland, Florida


 bool prevTrigState = 0;
 bool currTrigState = 0;
 unsigned long lastTriggerUp = 0;
 int delayReduction = 0;
 int stepsToGo; 
 unsigned long startSpeed = 400; //us 
 unsigned long shiftSpeed = 150; //us
 unsigned long runSpeed   = 125; //us
 double accelPhaseOne = 0.000000253494; //m value for ramp in low speed region
 double accelPhaseTwo = 0.000000180748; //m value for ramp in high speed region
 double decel =         0.000000439063; //m value for hard decel
 bool firstRun = 1;
 bool boltHomed = 0;
 double currSpeed = startSpeed; //to be used by fire() to be aware of motor speed from last run and mate speed ramp to that
 //unsigned long microsLastCommutation = 0; //try to maintain timing through latencies between fire() instances (not implemented seems to run real smooth without)
 double stepdelay; //us
 
 void commutate(double commDelay){
    //function to commutate the stepper once. note- immediate rising edge, trailing delay
    //consider replacing digitalWrites (~56 instructions) with PORTD |= 0b00001000 (PD3 is the step pin) (~2 instructions) may gets a bit faster.
    digitalWrite(3, HIGH);
    delayMicroseconds((unsigned long)(commDelay/2));
    digitalWrite(3, LOW);
    delayMicroseconds((unsigned long)(commDelay/2));
} 
 
 bool decelerateBoltToSwitch(){
  //try to gracefully end drivetrain rotation
  //called after the last fire() has returned
  //return true for home and false for not home (TBD)
  
  //fire() runs the bolt forward 720 clicks (on 4:1 mode) leaving us 80 to bring it down to ~3rps where we know it can stop instantly and keep position.
  //abort instantly on limit switch low (replace the constant 1 with the pin register when installed and wired up and the 80 with 90-100)
  stepsToGo = 150;
  stepdelay = currSpeed;
  while((stepsToGo > 0) && (PIND & 0b00010000)){
    commutate(stepdelay);
    if(stepdelay<startSpeed) {stepdelay = stepdelay*(1+decel*stepdelay*stepdelay);}
    stepsToGo--;
  }  
  currSpeed = startSpeed;
  boltHomed = 1;
  return !(PIND & 0b00010000);
}

bool reverseBoltToSwitch(){
  //this is called if decelerateBoltToSwitch() returns false
  stepsToGo = 800; //up to a full rev permitted (TBD)
  //set bolt direction reverse
  digitalWrite(2, HIGH);
  //run bolt back at idle speed
  while((PIND & 0b00010000)){
    commutate(startSpeed);
    stepsToGo--;
    if(stepsToGo == 0){
      //ran out of angle, die and reset direction
      digitalWrite(2, LOW);
      return false;
    }
  }
  digitalWrite(2, LOW);
  return !(PIND & 0b00010000);
}

void fire(){
  //loop called to fire a shot
  
  //set distance to run bolt forward
  stepsToGo = 720;
  //if continuing a previous instance add 80 steps
  if(!boltHomed) {stepsToGo += 80;}
  
  //get start point for first ramp
  if(currSpeed < startSpeed){
    //bolt already running
    stepdelay = currSpeed;
  } else {
    //bolt not running
    stepdelay = startSpeed;
  }
  // do first ramp if speed below shiftpoint
  while(stepdelay > shiftSpeed){
      commutate(stepdelay); 
      stepdelay = stepdelay*(1-accelPhaseOne*stepdelay*stepdelay);
      stepsToGo--;
  }
  //do second speed ramp if speed above shift but below running speed
  while(stepdelay > runSpeed){
    commutate(stepdelay);
    stepdelay = stepdelay*(1-accelPhaseTwo*stepdelay*stepdelay);
    stepsToGo--;
  }
  //do constant speed run until out of steps
  while(stepsToGo > 0){
    commutate(stepdelay);
    stepsToGo--;
  }
  currSpeed = stepdelay;
  boltHomed = 0;
}

void setup(){
 
  //pin 11 and 12 trigger switch
  //11 outside connector pin (default high)
  //12 inside connector pin (default low)
  pinMode(11, INPUT);
  pinMode(12, INPUT);
 
  //pin 10 flywheel motor controller PWM throttle signal
  pinMode(10, OUTPUT);
  //fast PWM prescaler 64 (250kHz)
  TCCR1A = _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  ICR1 = 624; //400Hz
  OCR1B = 230; //write 920us low throttle
  
  //bolt drive hardware
  //DRV8825 stepper driver card
  //pin 2: direction
  pinMode(2, OUTPUT);
  //drive low
  digitalWrite(2, LOW);
  //pin 3: step
  pinMode(3, OUTPUT);
  //pin 4: bolt limit switch (pullup)
  pinMode(4, INPUT);
  //pin 5,6,7: microstep mode select M2,M1,M0
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  //pin 8: enable
  pinMode(8, OUTPUT);
  //turn motor current off
  digitalWrite(8, HIGH);
  
  //configure stepper driver parameters
  //test config 4:1 microstep. TBD
  digitalWrite(5, LOW);
  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);
 
}

void loop(){
  if(firstRun) {
    //force bolt home from unknown state
    //turn motor current on
    digitalWrite(8, LOW);
    //avoid undefined 8825 behavior
    delay(20);
    //run back at 300rpm
    reverseBoltToSwitch();
    //turn motor current off
    digitalWrite(8, HIGH);
    //clear flag
    firstRun = 0;
  }
  
  //initial debounce on trigger from idle state. Safety measure.
  prevTrigState = currTrigState;
  currTrigState = (digitalRead(11) && !digitalRead(12));
  if(currTrigState && prevTrigState){
    //turn motor current on
    digitalWrite(8, LOW);
    //start flywheels
    OCR1B = 500; //go
    //wait for motor startup
    delay(100 - delayReduction);
    delayReduction = 20;
    fire();
    //first sealed-in shot is over. Check trigger *quickly* for downness, fire again and again while down.
    while((PINB & 0b00001000) && !(PINB & 0b00010000)){
     fire(); 
    }
    if(!decelerateBoltToSwitch()) {reverseBoltToSwitch();}
    lastTriggerUp = millis();
  } else {
    OCR1B = 230; //shutdown
    if((millis() - lastTriggerUp)>1000){
      //idle for more than 1s turn bolt motor current off
      digitalWrite(8, HIGH);
      //and reset to 100ms feed delay
      delayReduction = 0;
    }
  }
  delay(5);
}
