#include <ax12.h>
#include <avr/pgmspace.h>

//Initialize Variables.
int MotorID = -1;
int LEDDelay = 100;
int CommandType = -1;
int NumMotors = -1;
int NumCommandsLow = -1;
int NumCommandsHigh = -1;
int MoveDelay = 20;
int ReadDelay = 20;

void setup()
{
  //Initialize serial communication with baud rate of 9600.
//  Serial.begin(9600);
  Serial.begin(115200);

  //Set the built-in LED to be active.
  pinMode(0, OUTPUT);

}

void loop()
{
  //While the loop is running, have the LED blink.
  digitalWrite(0, HIGH);
  delay(LEDDelay);
  digitalWrite(0, LOW);
  delay(LEDDelay);

  //Check for bytes in the queue and respond approperiately.
  if (Serial.available() > 0)    //If there are any bytes in the queue...
  { 

    //Turn the LED on.
    digitalWrite(0, HIGH);

    //Read in the desired command type.
    CommandType = Serial.read();

    // Execute the motor commands.
    if (CommandType == 1)          // If the Command Type is set to 1...
    {
      //Write the positions to the motor.      
      MotorWrite();
    }

    // Read the motor positions.
    if (CommandType == 2)
    {
      //Read the positions of the motors.
      MotorRead();      
    }

    //Turn off the LED.
    digitalWrite(0, LOW);

  } 

}


void MotorWrite()
{

  //Read in the number of motors to move.
  while (Serial.available() < 1) {digitalWrite(0, LOW);}
  digitalWrite(0, HIGH);
  NumMotors = Serial.read();
  delay(ReadDelay);
  
  //Read in the low byte of the number of commands to be sent to the motors.
  while (Serial.available() < 1) {digitalWrite(0, LOW);}
  digitalWrite(0, HIGH);
  NumCommandsLow = Serial.read(); //NEED TO READ IN TWO BYTES IN CASE THE NUMBER IS VERY LARGE.
  delay(ReadDelay);
  
  //Read in the high byte of the number of commands to be sent to the motors.
  while (Serial.available() < 1) {digitalWrite(0, LOW);}
  digitalWrite(0, HIGH);
  NumCommandsHigh = Serial.read(); //NEED TO READ IN TWO BYTES IN CASE THE NUMBER IS VERY LARGE.
  delay(ReadDelay);
  
  //Preallocate an array to store the Motors IDs.
  int M[NumMotors];

  //Read in all of the Motor IDs from Matlab.
  for (int i2 = 0; i2 < NumMotors; i2++) {
    //Read in the motor ID.
    while (Serial.available() < 1) {digitalWrite(0, LOW);}
    M[i2] = Serial.read();
  }
  
  //Preallocate arrays to store the Position and Velocity Data.
  int Plow;
  int Phigh;
  int Vlow;
  int Vhigh;
  int MaxSize = 100;
  int Pmat[NumMotors][NumCommandsLow + 256*NumCommandsHigh];
  int Vmat[NumMotors][NumCommandsLow + 256*NumCommandsHigh];
//  int Pmat[NumMotors][MaxSize];
//  int Vmat[NumMotors][MaxSize];
//  int i3 = 0;
//  int i4 = 0;
  
  
  //Read in all of the Positions and Velocities from Matlab.
  for (int i1 = 0; i1 < (NumCommandsLow + 256*NumCommandsHigh); i1++) {
    
    for (int i2 = 0; i2 < NumMotors; i2++) {
      
      digitalWrite(0, HIGH);
      
      //Read in the motor position low byte.
      while (Serial.available() < 1) {digitalWrite(0, LOW);}
      Plow = Serial.read();

      //Read in the motor position high byte.
      while (Serial.available() < 1) {digitalWrite(0, LOW);}
      Phigh = Serial.read();

      //Read in the motor velocity low byte.
      while (Serial.available() < 1) {digitalWrite(0, LOW);}
      Vlow = Serial.read();

      //Read in the motor velocity high byte.
      while (Serial.available() < 1) {digitalWrite(0, LOW);}
      Vhigh = Serial.read();

      //Assemble the values into matrices.
      Pmat[i2][i1] = Plow + 256*Phigh;
      Vmat[i2][i1] = Vlow + 256*Vhigh;
        
    }
    
//    //Advance the counter.
//    i3++;
    
    delay(ReadDelay);
  }
  
  //Blink for debugging.
  for (int i = 0; i < 3; i++){
    //While the loop is running, have the LED blink.
    digitalWrite(0, HIGH);
    delay(LEDDelay + 500);
    digitalWrite(0, LOW);
    delay(LEDDelay + 500);
  }

  //Write the Motor IDs, Positions, and Velocities to the Motors.
  for (int i1 = 0; i1 < (NumCommandsLow + 256*NumCommandsHigh); i1++) {
    
    for (int i2 = 0; i2 < NumMotors; i2++) {
      //Set the motor velocity.
      ax12SetRegister2(M[i2], 32, Vmat[i2][i1]);

    }
    
    //Use a negligible delay to ensure that the motor positions are sent.
    delay(MoveDelay);
    
    for (int i2 = 0; i2 < NumMotors; i2++) {
      //Set the motor position.
      SetPosition(M[i2], Pmat[i2][i1]);
    }
    
    //Use a negligible delay to ensure that the motor positions are sent.
    delay(MoveDelay);
    
  }


  //Write to Matlab that the MotorWrite operation is complete.
  Serial.write(1);
  
}


void MotorRead()
{

  //Wait for the number of motors to be sent from Matlab.
  while (Serial.available() < 1) {}

  //Read in the number of motors to move.
  NumMotors = Serial.read();

  //Wait for all of the motor IDs to be sent.
  while (Serial.available() < NumMotors) {}

  //Write the current position of each motor to Matlab.
  for(int i = 1; i <= NumMotors; i++)      //Iterate through each motor...
  {
    //Retrieve the current Motor ID.
    MotorID = Serial.read();

    //Read the ACTUAL position of the motor and send it to Matlab.
    Serial.write(ax12GetRegister(MotorID, 36, 1));
    Serial.write(ax12GetRegister(MotorID, 37, 1));
    delay(ReadDelay);
  }  

}



