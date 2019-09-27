#include <stdio.h>            // Recommended over iostream for saving space
#include <propeller.h>        // Propeller-specific functions

#include "simpletools.h"
#include "abdrive.h"
#include "fdserial.h"

const char STOP = '9';
const char FORWARD = '0';
const char RIGHT_TURN = '1';
const char LEFT_TURN = '7';
const char SMALL_FORWARD = 'f';
const char SMALL_BACKWARD = 'b';
const char LEFT_PIVOT = '6';
const char RIGHT_PIVOT= '2';
const char CUSTOM_MOVE = 'c';



int distLeft[4], distRight[4];

int pathLeft[20], pathRight[20];

int traveledLeft, traveledRight;
int totalLeft, totalRight;
int currentStep = 0;
int lastStep = 0;



int getInt(fdserial *pi){
    char buffer[sizeof(int)];
    int temp = 0;

    while(fdserial_rxReady(pi) == 0);

    for(int i = 0; i < sizeof(int); i++){
        buffer[i] = fdserial_rxChar(pi);

    }

    sscanf(buffer, "%d", &temp);
    return temp;
}

int sendInt(fdserial *pi, int temp){
    char buffer[sizeof(int)];

    sprintf(buffer, "%d", temp);

    for(int i = 0; i < sizeof(int); i++){
        fdserial_txChar(pi, buffer[i]);
    }
    return temp;
}


int main()                                    // main function
{

    int partialMoveCounter = 0;

    char moves[100];
    for(int i = 0; i < 100; i++){
        moves[i] = STOP;
    }
    int currentMoveIndex = 0;
    int lastMoveIndex = 0;
    int immediateMove = -1;

    int prevLeftTicks = 0;
    int prevRightTicks = 0;
    
    int pathComplete = 0;

  

    /* start device */
    fdserial *term = fdserial_open(31,30,0,115200);

    while(1){

        int setPath = input(3);                    // P3 input -> button variable
        //print("setPath = %d\n", setPath);           // Display button state
        int goPin = input(4);                    // P4 input -> button variable
        // print("goPin = %d\n", goPin);           // Display button state
        int clearPath = input(5);                    // P5 input -> button variable
        //print("clearPath = %d\n", clearPath);           // Display button state



        if( setPath && !clearPath ){
            fdserial_txChar(term, 'c');


            int motor1 = getInt(term);
            int motor2 = getInt(term);
            //  print(" motor1 = %d", motor1);
            //  print(" motor2 = %d", motor2);
            drive_goto(motor1, motor2);

            drive_getTicks(&motor1, &motor2);

            int leftDif;
            int rightDif;
            if(  prevLeftTicks != 0 && prevRightTicks != 0){
                leftDif = motor1 - prevLeftTicks;
                rightDif = motor2 - prevRightTicks;
            }
            else{
                leftDif = motor1;
                rightDif = motor2;
            }
            prevLeftTicks = motor1;
            prevRightTicks = motor2;
            fdserial_txChar(term, immediateMove);

            sendInt(term, leftDif);
            sendInt(term, rightDif);




        }


        if( setPath && clearPath ) {
            fdserial_txChar(term, 's');
            currentStep = 0;
            lastStep = 0;
            pathComplete = 0;

            while( immediateMove != 0 ){
               //wait until data is available
                while(fdserial_rxReady(term) == 0);
                pathLeft[lastStep] = getInt(term);
                pathRight[lastStep] = getInt(term);

                immediateMove = pathLeft[lastStep];
               
                lastStep++;
                sendInt(term, lastStep);
                
            }

            //ignore the q that was sent to terminate the path

            lastStep--;

            //writeStr(term,"out of while ! = q \n");

            //transmit the total number of steps received
            //back to the raspberry pi
            sendInt(term, lastStep);

            immediateMove = getInt(term);
            // The following block is a useful bit of debugging code to dump the received path to a console.
            /*
              int temp = 0;
             while(temp<lastMoveIndex){
               writeStr(term,"moves = ");
                writeChar(term, moves[temp]);
                writeStr(term, "\n");
                temp++;
            //    writeChar(term, );
              //
               //
              }
            */

        }//end of if setPath and clearPath




        //drive_setRampStep(10);                      // 10 ticks/sec / 20 ms

        //drive_ramp(128, 128);


        while(goPin )                                    // Endless loo  p
        {

            while(currentStep < lastStep){

                if(pathLeft[currentStep] == pathRight[currentStep]){
                    while(traveledLeft != pathLeft[currentStep] && traveledRight != pathRight[currentStep]){
                        drive_getTicks(&distLeft[0], &distRight[0]);

                        // print("distLeft[0] = %d, distRight[0] = %d\n", distLeft[0], distRight[0]);

                        drive_speed(100, 100);
                      //  pause(1);


                        drive_getTicks(&distLeft[1], &distRight[1]);

                        // print("distLeft[1] = %d, distRight[1] = %d\n", distLeft[1], distRight[1]);

                        traveledLeft += distLeft[1] - distLeft[0];
                        traveledRight += distRight[1] - distRight[0];
                        //print("traveledLeft = %d, traveledRight = %d\n", traveledLeft, traveledRight);
                    }
                }

                else{
                    while(traveledLeft != pathLeft[currentStep] && traveledRight != pathRight[currentStep]){
                        drive_getTicks(&distLeft[0], &distRight[0]);

                        // print("distLeft[0] = %d, distRight[0] = %d\n", distLeft[0], distRight[0]);

                        drive_speed(pathLeft[currentStep] , pathRight[currentStep]);
                 //       pause(1);


                        drive_getTicks(&distLeft[1], &distRight[1]);

                        // print("distLeft[1] = %d, distRight[1] = %d\n", distLeft[1], distRight[1]);

                        traveledLeft += distLeft[1] - distLeft[0];
                        traveledRight += distRight[1] - distRight[0];
                        //print("traveledLeft = %d, traveledRight = %d\n", traveledLeft, traveledRight);
                    }
                }
//    if(pathLeft[currentStep] < 0 || pathRight[currentStep] < 0){
//      drive_speed(0,0);
//      pause(1);
//    }
                // print("traveledLeft = %d, traveledRight = %d\n", traveledLeft, traveledRight);
                currentStep++;
                totalLeft += traveledLeft;
                totalRight += traveledRight;
                traveledLeft = 0;
                traveledRight = 0;


            }
            drive_ramp(0, 0);
            //TODO:
            //send back a message to let the pi know the path has been completed
            // print("totalLeft = %d, totalRight = %d\n", totalLeft, totalRight);
            if(pathComplete == 0){
             sendInt(term, currentStep);
              pathComplete = 1;
            }              

            goPin = input(4);
            // print("imediateMove = %c\n", immediateMove);
        }//end of while goPin
        drive_ramp(0,0);
    }// end of while(1)
// Close the serial connection.
    fdserial_close(term);
// end of main
}