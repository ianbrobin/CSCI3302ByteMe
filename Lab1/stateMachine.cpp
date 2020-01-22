#include <Sparki.h>

class StateMachine
{
private:
  int currentState = 0;

  void init()
  {
   currentState = 0;
   sparki.gripperOpen();
   sparki.servo(SERVO_CENTER); 
  }

  void printState()
  {
      sparki.clearLCD();
      sparki.print("State => ");
      sparki.println(currentState);
      sparki.updateLCD();
  }

  void task1()
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
  }

  void task2()
  {
    sparki.moveForward();
    int cm = sparki.ping();

    if(cm < 7)
    {
      currentState += 1;
      printState();
    }
  }

  void task3()
  {
    sparki.gripperClose();

    currentState += 1;
    printState();
  }

  void task4()
  {
    sparki.moveStop();
  }


public:
  StateMachine()
  {
    init();
  }


  void TryNext()
  {
    switch(currentState)
    {
      case 0:
      {
        task1();
        break;
      }
      case 1:
      {
        task2();
        break;
      }
      case 2:
      {
        task3();
        break;
      }
      case 3:
      {
        task4();
        break;
      }
    }
    
    
  }
  
};
