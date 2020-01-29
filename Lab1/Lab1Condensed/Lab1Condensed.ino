#include <Sparki.h>

int counter;
int currentState;

void printState() {
      sparki.clearLCD();
      sparki.print("State => ");
      sparki.println(currentState);
      sparki.updateLCD();
}

void setup() {
  // put your setup code here, to run once:

  sparki.gripperOpen();
  sparki.servo(SERVO_CENTER); 

  currentState = 0;
  counter = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  switch(currentState)
    {
      case 0: {
        sparki.moveRight();
        delay(500);
        int cm = sparki.ping();
        sparki.clearLCD();
        sparki.println(cm);
        sparki.updateLCD();
        delay(250);
    
    
        if(cm < 30)
        {
          currentState += 1;
          printState();
        }
        break;
      }
        
      case 1: {
        sparki.moveForward();
        delay(250);
        int cm = sparki.ping();
    
        if(cm < 3)
        {
          sparki.moveStop();
          delay(5000);
          currentState += 1;
          printState();
        }
      }
        
      case 2: {
        sparki.gripperClose();
        delay(5000);
    
        currentState += 1;
      }
          
      case 3: {
        sparki.moveRight();
        delay(5000);

        currentState += 1;
        printState();
      }
      
      case 4: {
        sparki.moveStop();
        printState();
        delay(5000);
        int threshold = 700;
  
        int lineLeft   = sparki.lineLeft();   // measure the left IR sensor
        int lineCenter = sparki.lineCenter(); // measure the center IR sensor
        int lineRight  = sparki.lineRight();  // measure the right IR sensor
        
        if ( lineLeft >= threshold || lineCenter >= threshold || lineRight >= threshold) // move until line is found
        {  
         sparki.moveForward(); // move forward
         delay(1000);
        }
        else
        {
          sparki.moveStop();
          currentState += 1;
          printState();  
        }
      }

      case 5: {
        int threshold = 700;
    
        int lineLeft   = sparki.lineLeft();   // measure the left IR sensor
        int lineCenter = sparki.lineCenter(); // measure the center IR sensor
        int lineRight  = sparki.lineRight();  // measure the right IR sensor
        
        if ( lineCenter < threshold ) // if line is below left line sensor
        {  
          sparki.moveForward(); // move forward
          delay(1000);
        }
        else
        {
          if ( lineLeft < threshold ) // if line is below left line sensor
          {  
            sparki.moveLeft(); // turn left
            delay(1000);
          }
        
          if ( lineRight < threshold ) // if line is below right line sensor
          {  
            sparki.moveRight(); // turn right
            delay(1000);
          }    
        }
    
        //found start
        if(lineCenter < threshold && lineLeft < threshold && lineRight < threshold)
        {
          sparki.moveStop();
          currentState += 1;
          printState();  
        }
      }

      
      case 6: {
        sparki.beep();
        sparki.gripperOpen();
        delay(500);
    
        currentState += 1;
        printState();  
      }

      
      default: 
      {
        currentState = 60;
        sparki.clearLCD();
        sparki.println("Finished");
        sparki.updateLCD();
        sparki.moveStop();
        break;
      }
    }



  delay(100);
}
