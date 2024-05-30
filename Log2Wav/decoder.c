#include <stdlib.h>
#include "decoder.h"
#include "MsgProcessor.h"

unsigned char CalculateChecksum(int msgFunction,
                int msgPayloadLength, unsigned char msgPayload[])
{
    unsigned char checksum = 0;
    checksum ^= (unsigned char)0xFE;
    checksum ^= (unsigned char)(msgFunction >> 8);
    checksum ^= (unsigned char)(msgFunction >> 0);
    checksum ^= (unsigned char)(msgPayloadLength >> 8);
    checksum ^= (unsigned char)(msgPayloadLength >> 0);
    for (int i = 0; i < msgPayloadLength; i++)
    {
        checksum ^= msgPayload[i];
    }
    return checksum;
}

typedef enum StateReception_e
{
    Waiting,
    FunctionMSB,
    FunctionLSB,
    PayloadLengthMSB,
    PayloadLengthLSB,
    Payload,
    CheckSum
}StateReception;

StateReception rcvState = Waiting;
int msgDecodedFunction = 0;
int msgDecodedPayloadLength = 0;
unsigned char *msgDecodedPayload;
int msgDecodedPayloadIndex = 0;

//Debug stats
unsigned int msgDecoded = 0;

void DecodeMessage(unsigned char c, FILE* sensorFile)
{
        switch (rcvState)
        {
            case Waiting:
                if (c == 0xFE)
                    rcvState = FunctionMSB;
                break;
            case FunctionMSB:
                msgDecodedFunction = (short)(c << 8);
                rcvState = FunctionLSB;
                break;
            case FunctionLSB:
                msgDecodedFunction += (short)(c << 0);
                rcvState = PayloadLengthMSB;
                break;
            case PayloadLengthMSB:
                msgDecodedPayloadLength = (unsigned short)(c << 8);
                rcvState = PayloadLengthLSB;
                break;
            case PayloadLengthLSB:
                msgDecodedPayloadLength += (unsigned short)(c << 0);
                if (msgDecodedPayloadLength > 0)
                {
                    if (msgDecodedPayloadLength < 1024)
                    {
                        msgDecodedPayloadIndex = 0;
                        msgDecodedPayload = malloc(msgDecodedPayloadLength);
                        rcvState = Payload;
                    }
                    else
                    {
                        rcvState = Waiting;
                    }
                }
                else
                    rcvState = CheckSum;
                break;
            case Payload:
                if (msgDecodedPayloadIndex > msgDecodedPayloadLength)
                {
                    //Erreur
                    msgDecodedPayloadIndex = 0;
                    rcvState = Waiting;
                }
                msgDecodedPayload[msgDecodedPayloadIndex++] = c;
                if (msgDecodedPayloadIndex >= msgDecodedPayloadLength)
                {
                    rcvState = CheckSum;
                    msgDecodedPayloadIndex = 0;
                }
                break;
            case CheckSum:
            {
                unsigned char calculatedChecksum = CalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
                unsigned char receivedChecksum = c;
                if (calculatedChecksum == receivedChecksum)
                {
                    //Lance l'event de fin de decodage
                    ProcessDecodedMessage(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload,sensorFile);
                    msgDecoded++;
                }
                else
                {
                    //printf("erreur Checksum");
                }
                rcvState = Waiting;
            }
                break;
            default:
                rcvState = Waiting;
                break;
        }
}