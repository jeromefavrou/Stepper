#include "Stepper.hpp"

Stepper BM_stepper(48,120.0*141.0*(140.0/27.0)); //(nb_steps , reductor factor )

void setup() 
{
  BM_stepper.max_freq(250);
  BM_stepper.min_freq(0);
  BM_stepper.speed_rpm(1.0/(60.0*24.0));
  short pins_a[2]={8,6};
  short pins_b[2]={7,5};
  BM_stepper.init<2,2>(pins_a,pins_b,2);
  
  BM_stepper.direction(Stepper::DIRECTION::FORWARD);
  BM_stepper.move_steps(500);
  
  delay(1000);
  
  BM_stepper.direction(Stepper::DIRECTION::BACKWARD);
  BM_stepper.move_steps(500);
  
  delay(50);

  BM_stepper.start();
}

void loop() 
{
  BM_stepper.move_async(Stepper::MOVE_TYPE::HIGH_TORQUE);

  delay(3);
}

