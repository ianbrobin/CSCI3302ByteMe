#include <Sparki.h>
#include "stateMachine.cpp"


StateMachine sm;

void setup() {
  // put your setup code here, to run once:
  //sparki.ping();
  
  
  sm = StateMachine();
}


void loop() {
  // put your main code here, to run repeatedly

  sm.TryNext();
  delay(1000);

}
