#include "main.hpp"
#include <Arduino.h>
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

void processMessage(const unsigned char *message, L298NX2 &firstBoard, L298NX2 &secondBoard, Adafruit_PWMServoDriver &pwm)
{
    String msgString = String((const char *)message);

    // Tokenize and process the message
    char msgArray[msgString.length() + 1];
    msgString.toCharArray(msgArray, msgString.length() + 1);

    char *token = strtok(msgArray, " ");

    while (token != nullptr)
    {
        String name = token;
        token = strtok(nullptr, " ");

        if (token != nullptr)
        {
            float value = String(token).toFloat();
            int speed = constrain(abs(value), MIN_SPEED_WHEEL, MAX_SPEED_WHEEL);
            int pwmValue = map(speed, MIN_SPEED_WHEEL, MAX_SPEED_WHEEL, LOW_PWM, HIGH_PWM);
            bool isBackward = (value < 0);
            bool isForward = (value > 0);

#if SERIAL_ENABLE == 1
            Serial.print("Name: ");
            Serial.print(name);
            Serial.print(" Value: ");
            Serial.println(value);
            // Serial.print(" Position: ");
            // Serial.println(position);
            // Serial.print(" Direction: ");
            // Serial.println(isBackward ? "BACKWARD" : "FORWARD");
#endif

            if (name == "fr,")
            { // For the FRONT RIGHT motor
                firstBoard.setSpeedA(pwmValue);
                if (isBackward)
                {
                    firstBoard.runA(L298N::BACKWARD); // if value is negative reverse
                }
                else if (isForward)
                {
                    firstBoard.runA(L298N::FORWARD); // else go forward
                }
                else
                {
                    firstBoard.runA(L298N::STOP);
                }
            }
            else if (name == "fl,")
            { // For the FRONT LEFT motor
                firstBoard.setSpeedB(pwmValue);
                if (isBackward)
                {
                    firstBoard.runB(L298N::BACKWARD); // if value is negative reverse
                }
                else if (isForward)
                {
                    firstBoard.runB(L298N::FORWARD); // else go forward
                }
                else
                {
                    firstBoard.runB(L298N::STOP);
                }
            }
            else if (name == "rr,")
            { // For the REAR RIGHT motor
                secondBoard.setSpeedA(pwmValue);
                if (isBackward)
                {
                    secondBoard.runA(L298N::BACKWARD); // if value is negative reverse
                }
                else if (isForward)
                {
                    secondBoard.runA(L298N::FORWARD); // else go forward
                }
                else
                {
                    secondBoard.runA(L298N::STOP);
                }
            }
            else if (name == "rl,")
            { // For the REAR LEFT motor
                secondBoard.setSpeedB(pwmValue);
                if (isBackward)
                {
                    secondBoard.runB(L298N::BACKWARD); // if value is negative reverse
                }
                else if (isForward)
                {
                    secondBoard.runB(L298N::FORWARD); // else go forward
                }
                else
                {
                    secondBoard.runA(L298N::STOP);
                }
            }
            else if (name == "1,")
            {
                // Pinky Finger
                int angle = map((abs(value) + 1.57), 1.57, 3.14, 0, 180);
                int position = map(angle, 0, 180, 480, 150);
                pwm.setPWM(3, 0, position);
            }
            else if (name == "2,")
            {
                // Ring Finger
                int angle = map((abs(value) + 1.57), 1.57, 3.14, 0, 180);
                int position = map(angle, 0, 180, 480, 150);
                pwm.setPWM(4, 0, position);
            }
            else if (name == "3,")
            {
                // Middle Finger
                int angle = map((abs(value) + 1.57), 1.57, 3.14, 0, 180);
                int position = map(angle, 0, 180, 480, 100);
                pwm.setPWM(2, 0, position);
            }
            else if (name == "4,")
            {
                // Index Finger
                float angle = map((abs(value) + 1.57), 1.57, 3.14, 0, 180);
                int position = map(angle, 0, 180, 100, 450);
                pwm.setPWM(1, 0, position);
            }
            else if (name == "5,")
            {
                // Thumb
                int angle = map((abs(value) + 1.57), 1.57, 3.14, 0, 180);
                int position = map(angle, 0, 180, 480, 150);
                pwm.setPWM(0, 0, position);
            }
            else if (name == "6,")
            {
                // Arm
                int angle = map((abs(value) + 1.57), 1.57, 3.14, 0, 180);
                int position = map(angle, 0, 180, SERVOMIN, SERVOMAX);
                pwm.setPWM(5, 0, position);
                pwm.setPWM(6, 0, position);
            }
            else if (name == "7,")
            {
                // Wrist
                int angle = map((abs(value) + 1.57), 1.57, 3.14, 0, 180);
                int position = map(angle, 0, 180, SERVOMIN, SERVOMAX);
                pwm.setPWM(7, 0, position);
            }
        }
        token = strtok(nullptr, " ");
    }
}