#include "../lib/main.hpp"
/**
 * tokenizetoint - helps with converting the input to arrays
 * @input: input from the ROS nodes
 * Return: pointer to an array of integers
 */
int *tokenizetoint(String input)
{
    int *commands = new int[MAX_COMMANDS];
    int index = 0;
    int start = 0;
    int end = input.indexOf(' ');

    while (end != -1 && index < MAX_COMMANDS)
    {
        String atoken = input.substring(start, end);
        commands[index] = atoken.toInt(); // Convert to integer
        index++;
        start = end + 1;
        end = input.indexOf(' ', start);
    }

    if (index < MAX_COMMANDS)
    {
        String atoken = input.substring(start);
        commands[index] = atoken.toInt(); // Convert to integer
        index++;
    }
    return (commands);
}