#include "main.hpp"
/*
* processMessage - Tokenizes String and writes name and value to Serial
* message: String from SerialBT
* Return: Nothing 
*/

void processMessage(String message)
{
    // Print the received message
    Serial.print("Received: ");
    Serial.println(message);

    // Tokenize and process the message
    char msgArray[message.length() + 1];
    message.toCharArray(msgArray, message.length() + 1);

    char *token = strtok(msgArray, " ");
    while (token != nullptr)
    {
        String name = token;
        token = strtok(nullptr, " ");
        if (token != nullptr)
        {
            float value = atof(token);
            Serial.print("Name: ");
            Serial.print(name);
            Serial.print(" Value: ");
            Serial.println(value);
        }
        token = strtok(nullptr, " ");
    }
}
/*
* processMessage - Tokenizes String and writes motor speed values to the L298N
* message: String from SerialBT
* firstBoard: L298N motor driver 1
* secondBoard: L298N motor driver 2
* Return: Nothing 
*/

#define MAX_SPEED_WHEEL 21
#define MIN_SPEED_WHEEL 0
#define LOW_PWM 0
#define HIGH_PWM 255
void processMessage(String message, L298NX2 firstBoard, L298NX2 secondBoard)
{
    // process Message but write to the L298N
    // NOTE: MAX VELOCITY OF THE WHEELS - 21 rad/s (TTGEARED smh)

    unsigned int conVal = 0;
    // Tokenize and process the message
    char msgArray[message.length() + 1];
    message.toCharArray(msgArray, message.length() + 1);

    char *token = strtok(msgArray, " ");
    while (token != nullptr)
    {
        String name = token;
        token = strtok(nullptr, " ");
        if (token != nullptr)
        {
            float value = atof(token);
            Serial.print("Name: ");
            Serial.print(name);
            if (name == "fr,")
            { // For the first motor
                conVal = constrain(abs(value), MIN_SPEED_WHEEL, MAX_SPEED_WHEEL);
                firstBoard.setSpeedA(map(conVal, MIN_SPEED_WHEEL, MAX_SPEED_WHEEL, LOW_PWM, HIGH_PWM));

                if (value < 0)
                    firstBoard.backwardA(); // if value is negative reverse
                else
                    firstBoard.forwardA(); // else go forward
            }
            else if (name == "fl,")
            { // For the second motor
                conVal = constrain(abs(value), MIN_SPEED_WHEEL, MAX_SPEED_WHEEL);
                firstBoard.setSpeedB(map(conVal, MIN_SPEED_WHEEL, MAX_SPEED_WHEEL, LOW_PWM, HIGH_PWM));

                if (value < 0)
                    firstBoard.backwardB(); // if value is negative reverse
                else
                    firstBoard.forwardB(); // else go forward
            }
            else if (name == "rl,")
            { // For the third motor
                conVal = constrain(abs(value), MIN_SPEED_WHEEL, MAX_SPEED_WHEEL);
                secondBoard.setSpeedA(map(conVal, MIN_SPEED_WHEEL, MAX_SPEED_WHEEL, LOW_PWM, HIGH_PWM));
                
                if (value < 0)
                    secondBoard.backwardA(); // if value is negative reverse
                else
                    secondBoard.forwardA(); // else go forward
            }
            else if (name == "rr,")
            { // For the fourth motor
                conVal = constrain(abs(value), MIN_SPEED_WHEEL, MAX_SPEED_WHEEL);
                secondBoard.setSpeedB(map(conVal, MIN_SPEED_WHEEL, MAX_SPEED_WHEEL, LOW_PWM, HIGH_PWM));
                
                if (value < 0)
                    secondBoard.backwardB(); // if value is negative reverse
                else
                    secondBoard.forwardB(); // else go forward
            }
            else if (name == "1,")
            {
                //This is for the 2 servos at the base-forearm joint M966R
                //RUN SOME CODE HERE FOR THE SERVO
            }
            else if (name == "2,")
            {
                //Wrist Servo M966R
            }
            else if (name == "3,")
            {
                //Pinky Finger SG90s
            }
            else if (name == "4,")
            {
                //Ring Finger SG90
            }
            else if (name == "5,")
            {
                //Middle Finger SG90
            }
            else if (name == "6,")
            {
                //Index Finger SG90
            }
            else if (name == "7,")
            {
                //Thumb SG90
            }
            Serial.print(" Value: ");
            Serial.print(value);
            Serial.print(" || conVal: ");
            Serial.println(conVal);
        }
        token = strtok(nullptr, " ");
    }
}