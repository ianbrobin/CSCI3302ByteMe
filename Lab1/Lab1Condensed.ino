#include <Sparki.h>


int currentState = 0;

void setup() {
  // put your setup code here, to run once:
  //sparki.ping();
  sparki.clearLCD();
  sparki.print("test");
  sparki.updateLCD();
  
  currentState = 0;
  sparki.gripperOpen();
  sparki.servo(SERVO_CENTER); 
  
}


void loop() {
  // put your main code here, to run repeatedly
  //sparki.moveForward();
  sparki.clearLCD();
  sparki.print("State => ");
  sparki.println(currentState);
  sparki.updateLCD();

  switch(currentState)
    {
      case 0:
      {
        sparki.moveRight(1);

        int cm = sparki.ping();

        sparki.clearLCD();
        sparki.println(cm);
        sparki.updateLCD();

        if(cm < 30)
        {
          currentState += 1;
        }
        break;
      }
      case 1:
      {
        sparki.moveForward();
        int cm = sparki.ping();

        if(cm < 7)
        {
          currentState += 1;
        }
        break;
      }
      case 2:
      {
        sparki.gripperClose();

        currentState += 1;
        break;
      }
      case 3:
      {
        sparki.moveStop();
        break;
      }
    }
}
