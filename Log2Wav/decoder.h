#include <stdio.h>

unsigned char CalculateChecksum(int msgFunction,
                int msgPayloadLength, unsigned char msgPayload[]);
void DecodeMessage(unsigned char c, FILE* sensorFile);