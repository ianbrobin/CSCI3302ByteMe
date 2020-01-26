#include <Sparki.h>

class StateMachine
{
private:
  int currentState = 0;

  //reset sparki state
  void init()
  {
   currentState = 0;
   sparki.gripperOpen();
   sparki.servo(SERVO_CENTER); 
  }

  //print current state ID to LCD
  void printState()
  {
      sparki.clearLCD();
      sparki.print("State => ");
      sparki.println(currentState);
      sparki.updateLCD();
  }


  //Rotate right until an object is found within 30cm
  bool task0()
  {
    sparki.moveRight(1);

    int cm = sparki.ping();

    sparki.clearLCD();
    sparki.println(cm);
    sparki.updateLCD();

    if(cm < 30)
    {
      currentState += 1;
      printState();
    }

    return true;
  }

  //Move forward until object is within 7cm
  bool task1()
  {
    sparki.moveForward();
    int cm = sparki.ping();

    if(cm < 7)
    {
      currentState += 1;
      printState();
    }

    return true;
  }

  
  //Close hands
  bool task2()
  {
    sparki.gripperClose();

    currentState += 1;
    printState();

    return true;
  }


  //Rotate 180 degrees
  bool task3()
  {
    sparki.moveRight(180);

    currentState += 1;
    printState();   

    return true;
  }


  //Drive forward until sparki is on a line
  bool task4()
  {
    int threshold = 700;

    int lineLeft   = sparki.lineLeft();   // measure the left IR sensor
    int lineCenter = sparki.lineCenter(); // measure the center IR sensor
    int lineRight  = sparki.lineRight();  // measure the right IR sensor
    
    if ( lineLeft >= threshold || lineCenter >= threshold || lineRight >= threshold) // move until line is found
    {  
     sparki.moveForward(); // move forward
    }
    else
    {
      sparki.moveStop();
      currentState += 1;
      printState();  
    }

    return true;
  }


  //Follow line until sparki reaches the start marker
  bool task5()
  {
    int threshold = 700;

    int lineLeft   = sparki.lineLeft();   // measure the left IR sensor
    int lineCenter = sparki.lineCenter(); // measure the center IR sensor
    int lineRight  = sparki.lineRight();  // measure the right IR sensor
    
    if ( lineCenter < threshold ) // if line is below left line sensor
    {  
      sparki.moveForward(); // move forward
    }
    else
    {
      if ( lineLeft < threshold ) // if line is below left line sensor
      {  
        sparki.moveLeft(); // turn left
      }
    
      if ( lineRight < threshold ) // if line is below right line sensor
      {  
        sparki.moveRight(); // turn right
      }    
    }

    //found start
    if(lineCenter < threshold && lineLeft < threshold && lineRight < threshold)
    {
      sparki.moveStop();
      currentState += 1;
      printState();  
    }
    

    return true;
  }


  //Beep and open hands
  bool task6()
  {

    sparki.beep();
    sparki.gripperOpen();

    currentState += 1;
    printState();  

    return true;
  }


public:
  StateMachine()
  {
    init();
  }


  bool TryNext()
  {
    switch(currentState)
    {
      case 0: return task0();
      case 1: return task1();
      case 2: return task2();
      case 3: return task3();
      case 4: return task4();
      case 5: return task5();
      case 6: return task6();
      default: 
      {
        sparki.moveStop();
        return false;
      }
    }
    
    
  }
  
};
