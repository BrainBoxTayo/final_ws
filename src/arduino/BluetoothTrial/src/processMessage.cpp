#include "../lib/main.hpp"
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

void processMessage(String message, L298NX2 firstBoard, L298NX2 secondBoard)
{
    // process Message but write to the L298N

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
                conVal = constrain(abs(value), 0, 100);
                firstBoard.setSpeedA(map(conVal, 0, 100, 0, 240));

                if (value < 0)
                    firstBoard.backwardA(); // if value is negative reverse
                else
                    firstBoard.forwardA(); // else go forward
            }
            if (name == "fl,")
            { // For the second motor
                conVal = constrain(abs(value), 0, 100);
                firstBoard.setSpeedB(map(conVal, 0, 100, 0, 240));

                if (value < 0)
                    firstBoard.backwardB(); // if value is negative reverse
                else
                    firstBoard.forwardB(); // else go forward
            }
            if (name == "rl,")
            { // For the third motor
                conVal = constrain(abs(value), 0, 100);
                secondBoard.setSpeedA(map(conVal, 0, 100, 0, 240));
                
                if (value < 0)
                    secondBoard.backwardA(); // if value is negative reverse
                else
                    secondBoard.forwardA(); // else go forward
            }
            if (name == "rr,")
            { // For the fourth motor
                conVal = constrain(abs(value), 0, 100);
                secondBoard.setSpeedB(map(conVal, 0, 100, 0, 240));
                
                if (value < 0)
                    secondBoard.backwardB(); // if value is negative reverse
                else
                    secondBoard.forwardB(); // else go forward
            }
            Serial.print(" Value: ");
            Serial.print(value);
            Serial.print(" || conVal: ");
            Serial.println(conVal);
        }
        token = strtok(nullptr, " ");
    }
}