#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h> 


// define two tasks for Blink & AnalogRead
//void TaskBlink( void *pvParameters );
void TaskAnalogRead( void *pvParameters );
void TaskButton( void *pvParameters );

#define ROTARY_ANGLE_SENSOR A0
#define ADC_REF 5//reference voltage of ADC is 5v.If the Vcc switch on the seeeduino
                     //board switches to 3V3, the ADC_REF should be 3.3
#define GROVE_VCC 5//VCC of the grove interface is normally 5v
#define FULL_ANGLE 300//full value of the rotary angle is 300 degrees

const int numReadings = 10;     // number of data to stored
int readings[numReadings];      // 10 equal value
int readIndex = 0;              // the index of the current reading
int dataStored[numReadings];      // the last 10 equal value
int count = 1;                  // counter for value 
unsigned long minTime = 0;      // assign this time to the first element of the vector 'readings[]'
unsigned long MaxTime = 0;      // assign this time to the last element of the vector 'readings[]'
const int buttonPin = 2;        // the number of the pushbutton pin
int buttonState = 0;            // variable for reading the pushbutton status
int servoPin = 8;                // servo Pin 
QueueHandle_t Global_Queue_Handle = 0;
SemaphoreHandle_t button_signal = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  
  // Now set up two tasks to run independently.
  /*xTaskCreate(
    TaskBlink
    ,  (const portCHAR *)"Blink"   // A name just for humans
    ,  128  // Stack size
    ,  NULL
    ,  1  // priority
    ,  NULL );*/

  xTaskCreate(
    TaskAnalogRead
    ,  (const portCHAR *) "AnalogRead"
    ,  128 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  4  // priority
    ,  NULL );

  xTaskCreate(
    TaskButton
    ,  (const portCHAR *)"Button"   // A name just for humans
    ,  128  // Stack size
    ,  NULL
    ,  1  // priority
    ,  NULL );

  xTaskCreate(
    TaskMiniServo
    ,  (const portCHAR *)"Button"   // A name just for humans
    ,  128  // Stack size
    ,  NULL
    ,  2  // priority
    ,  NULL );

  //A queue must be explicitly created before it can be used
  Global_Queue_Handle = xQueueCreate(10, sizeof(int));

  //A semaphore must be explicitly created before it can be used
  vSemaphoreCreateBinary(button_signal);

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
  /*Start the scheduler*/
  /*vTaskStartScheduler();*/

  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

/*void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}*/


void TaskAnalogRead(void *pvParameters)  // This is TaskAnalogRead.
{
  (void) pvParameters;

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  //pinMode(ROTARY_ANGLE_SENSOR, INPUT);

  for (;;)
  {

    int degrees;
        degrees = getDegree();
        Serial.println("The angle between the mark and the starting position:");
        Serial.println(degrees);

   
    // read from the sensor:
    readings[readIndex] = degrees;

    //check if the last read is equal to current read
    if (readIndex == 0){
      count = 1;
      //when the first value coming I assign to 'minTime' the current time
      minTime = millis();
      Serial.println(minTime);
    }
    else if (readings[readIndex-1] == readings[readIndex]){ 
      count = count + 1;
      //when the last value coming I assign to 'MaxTime' the current time
      if(readIndex == 9){
        MaxTime = millis();
        Serial.println(MaxTime);
      }
    }else{
      count = 1;
      //when the first value coming I assign to 'minTime' the current time
      minTime = millis();
      Serial.println(minTime);
      // if just value is different then we must reset the vector
      for (int thisReading = 0; thisReading < numReadings; thisReading++) {
      readings[thisReading] = 0;
      }
      // reset index vector
      readIndex = 0;
      // add the value just readed
      readings[readIndex] = degrees;
    }
    
    // add index
    readIndex = readIndex + 1;
    Serial.println(count);

    //if there are 10 values equal in two seconds
    if ((count == 10) && ((MaxTime - minTime) >= 2000 &&(MaxTime - minTime) <= 2300)){
      Serial.println("Yeahhhh There are 10 same values!");
      Serial.println("In TWO SECONDS!");
      
      count = 0;
      readIndex = 0;
      int thisReading;
      // initialize all the readings to 0:
      for (thisReading = 0; thisReading < numReadings; thisReading++) {
      dataStored[thisReading] = readings[thisReading]; /*dataStored is the array that contain the last 10 equal value*/
      readings[thisReading] = 0;
      }
      dataStored[thisReading] = '\0';
      //TaskAnalogRead check if the signal has been given
      if(xSemaphoreTake(button_signal, 50)){
        Serial.println("The SEMAPHORE SIGNAL is arrived!");
        //Send value to queue
        int *p = &dataStored[0];
        if(! xQueueSend(Global_Queue_Handle, &p, 0)){
          Serial.println("Failed to send to queue\n");
        }
      }
    }

    
    vTaskDelay(15);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskButton(void *pvParameters)  // This is TaskButton.
{
  (void) pvParameters;
    
    // initialize the pushbutton pin as an input:
    pinMode(buttonPin, INPUT);  

  for(;;)
  {
     // read the state of the pushbutton value:
     buttonState = digitalRead(buttonPin);
     if (buttonState == HIGH) //If button is HIGH
        {
          xSemaphoreGive(button_signal); //give the signal to TaskAnalogRead   
        }
  }
}

void TaskMiniServo(void *pvParameters)  // This is TaskMiniServo.
{
  (void) pvParameters;

  //myServo.attach(servoPin); 
  pinMode(servoPin,OUTPUT); //Setting servo pin

  int *rx_dataStored;
  for (;;) // A Task shall never return or exit.
  {

    //Receive value from queue
    if( xQueueReceive(Global_Queue_Handle, &rx_dataStored, 50 )){
      Serial.println("Received data from queue");
        //Print each array's value
        int i = 0;
        do{
            Serial.println(*rx_dataStored);
            int angle = *rx_dataStored;
            servoPulse(servoPin, angle); // Function to move servo
            *rx_dataStored++;
            i++;
        }while(i < 10);
  }
     else{
            Serial.println("Failed to receive data from queue");
     }
  }
  vTaskDelay(100);
}

void servoPulse (int servo, int angle)
{
  int pwm = angle * 9 + 500;
  digitalWrite(servo, HIGH);
  delayMicroseconds(pwm); //First delay
  digitalWrite(servo, LOW);
  delay(50); //Second delay
}

int getDegree()
    {
        int sensor_value = analogRead(ROTARY_ANGLE_SENSOR);
        float voltage;
        voltage = (float)sensor_value*ADC_REF/1023;
        float degrees = (voltage*FULL_ANGLE)/GROVE_VCC;
        return degrees;
    }

