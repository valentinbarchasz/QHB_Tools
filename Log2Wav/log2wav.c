// April 2024
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "Macros.h"
#include "decoder.h"

#define MAX_PERIPHERAL 5                   //Nombre max de peripheriques externes


typedef struct{
    char Type;                                       //type du peripherique (0x01 accel, 0x02 gyro, 0x03 magneto, 0x04 temperature, 0x05 pressure, 0x06 light,...)
    char ID;                                         //ID du peripherique
    char Range;                                      //Range de la mesure (ex: 2G, 4G, 6G, 8G, 16G pour un accel)
    char Resolution;                                 //Resolution de mesure du peripherique
    short Frequency;                                 //Frequence d'echantillonage du peripherique
}PERIPHERAL_CONFIGURATION;

typedef struct{
    int headerSize;       //Taille du header ce champ exclu
    int versionNumber;
    char numberOfChan;
    char resolutionBits;
    int samplingFrequency;
    int dmaBlockSize;
    int sizeOfAdditionnalDataBuffer;
    char numberOfExternalPeripheral;
    int timeStampOfStart;
    PERIPHERAL_CONFIGURATION periphConfig[MAX_PERIPHERAL];
}HighBlueHeader;

struct WaveHeader_s {
    char chunkId[4]; // Riff Wave Header
    int  chunkSize;
    char format[4];
    char subChunk1Id[4]; // Format Subchunk
    int  subChunk1Size;
    short int audioFormat;
    short int numChannels;
    int sampleRate;
    int byteRate;
    short int blockAlign;
    short int bitsPerSample;
    //short int extraParamSize;
    char subChunk2Id[4]; // Data Subchunk
    int  subChunk2Size;
} WaveHeader_default = {{'R','I','F','F'}, 36, {'W','A','V','E'}, {'f','m','t',' '}, 16, 1, 0, 0, 0, 0, 0, {'d','a','t','a'}, 0};
typedef struct WaveHeader_s WaveHeader;


void parseLogFileHeader(FILE* logfile, HighBlueHeader* hdr, int verbose){
  // header parse
  fread(&hdr->headerSize, 4, 1, logfile);
  fread(&hdr->versionNumber, 2, 1, logfile);
  fread(&hdr->numberOfChan, 1, 1, logfile);
  fread(&hdr->resolutionBits, 1, 1, logfile);
  fread(&hdr->samplingFrequency, 4, 1, logfile);
  fread(&hdr->dmaBlockSize, 4, 1, logfile);
  fread(&hdr->sizeOfAdditionnalDataBuffer, 4, 1, logfile);
  fread(&hdr->numberOfExternalPeripheral, 1, 1, logfile);
  fread(&hdr->timeStampOfStart, 4, 1, logfile);
  if(verbose){
    printf("header size : %d\n", hdr->headerSize);
    printf("version number : %d\n", hdr->versionNumber);
    printf("number of chans : %d\n", hdr->numberOfChan);
    printf("resolutionBits : %d\n", hdr->resolutionBits);
    printf("samplingFrequency : %d\n", hdr->samplingFrequency);
    printf("dmaBlockSize : %d\n", hdr->dmaBlockSize);
    printf("sizeOfAdditionnalDataBuffer : %d\n", hdr->sizeOfAdditionnalDataBuffer);
    printf("numberOfExternalPeripheral : %d\n", hdr->numberOfExternalPeripheral);
    printf("timeStampOfStart : %d\n", hdr->timeStampOfStart);
  }
  // load external periph config
  for(int i=0; i<hdr->numberOfExternalPeripheral; i++){
      fread(&hdr->periphConfig[i].Type, 1, 1, logfile);
      fread(&hdr->periphConfig[i].ID, 1, 1, logfile);
      fread(&hdr->periphConfig[i].Range, 1, 1, logfile);
      fread(&hdr->periphConfig[i].Resolution, 1, 1, logfile);
      fread(&hdr->periphConfig[i].Frequency, 2, 1, logfile);
  }
  return;
}

short int toLittleEndian(short int val){
  return val = ((val & 0x00FF)<<8) | ((val & 0xFF00)>>8);
}

void parseMPU(unsigned char* additionnalDataBlock, int size, bool verbose, FILE* mpuFile){
  static int maxtimeStamp = 0;
  int i, timestamp;
  short int trameSize = 31, val; // fixed for now
  unsigned char* curData = additionnalDataBlock + 6; // first 6 bytes are for usb device
  if(verbose){
    printf("MPU Range : %hdG\n", *(curData + 5 + 3));
    printf("MPU Resolution : %hd\n", *(curData + 5 + 3 + 1));
    printf("MPU Sampling Frequency : %hd\n", *(curData + 5 + 3 + 3));
  }
  while(curData + trameSize + 6 < additionnalDataBlock + size){
    if(!(curData[0]==0xFE && curData[1]==0x0A && curData[2]==0x0A && curData[5]==0x08)){
      // skip trame if header is incorrect
      curData += trameSize + 6;
      continue;
    }
    curData += 3 + 2; // skip trame header, trame length
    timestamp = *((int*) (curData + 9));
    timestamp = ((timestamp & 0xFF000000)>>24) | ((timestamp & 0x00FF0000)>>8) | ((timestamp & 0x0000FF00)<<8) | ((timestamp & 0x000000FF)<<24);
    if(timestamp > maxtimeStamp){
      fprintf(mpuFile, "%d,", timestamp);
      //printf("%d\n", timestamp);
      // treat payload
      for(i=13; i<31; i+=2){
        val = *((short int*) (curData + i));
        val = ((val & 0x00FF)<<8) | ((val & 0xFF00)>>8);
        //printf("curData %x %x %hd", *(curData+i), *(curData + i +1), val);
        if(i<29){
          fprintf(mpuFile, "%hd,", val);
        }else{
          fprintf(mpuFile, "%hd\n", val);
        }
      }
      maxtimeStamp = timestamp;
    }
    curData += trameSize + 1; // shift of trame size + 1 byte of checksum
  }
}

unsigned char softwareMajorRev=0;
unsigned char softwareMinorRev=0;
int main(int argc, char* argv[]){
  if(argc < 2){
    printf("Script needs to be called with at least 1 arguments :\n\t input file name (.log file)\n\t(optionnal) output filename (.wav file)\n\t(optionnal) sensor Datas filename (.csv file)\n\t(optionnal) verbose (1 / void)\n");
    return 0;
  }
  HighBlueHeader hdr;
  FILE* logfile = fopen(argv[1], "rb");
  if(logfile==NULL){
    printf("Failed to open input file\n");
    return 0;
  }
  // get file size
  fseek(logfile, 0, SEEK_END);
  long filesize =  ftell(logfile);
  if(filesize == 0){
    printf("skipped empty file : %s\n", argv[1]);
    return 0;
  }
  fseek(logfile, 0, SEEK_SET);
  int verbose = 0;
  if(argc==5){
    verbose = *argv[4]=='1';
  }
  // file opened successfully, we read the header
  parseLogFileHeader(logfile, &hdr, verbose);
  int resolutionBytes = hdr.resolutionBits/8;
  long dataBlockSampleSize = hdr.dmaBlockSize / ( hdr.numberOfChan  * resolutionBytes);
  if(verbose){
    printf("file version number: %d,%d",softwareMajorRev,softwareMinorRev);
    printf("file size (bytes) %ld \n", filesize);
    printf("dataBlockSampleSize %ld\n", dataBlockSampleSize);
  }
  if(hdr.resolutionBits!=16 && hdr.resolutionBits!=24 && hdr.resolutionBits!=32){
    printf("resolution %d not supported yet sorry\n", hdr.resolutionBits);
    return 0;
  }
  // move to start of the data
  fseek(logfile, hdr.headerSize + 4, SEEK_SET);

  FILE* wavfile;// open wav file
  if(argc>2){
    wavfile = fopen(argv[2], "wb");
  }else{
    strcpy(argv[1] + strlen(argv[1])-3, "wav\0");
    wavfile = fopen(argv[1], "wb");
  }
  if(wavfile==NULL){
    printf("Failed to open wav output file\n");
    return 0;
  }
  WaveHeader whdr = WaveHeader_default;
  whdr.numChannels = hdr.numberOfChan;
  whdr.sampleRate = hdr.samplingFrequency;
  whdr.bitsPerSample = hdr.resolutionBits;
  whdr.byteRate = whdr.sampleRate * whdr.numChannels * resolutionBytes;
  whdr.blockAlign = whdr.numChannels * resolutionBytes;
  whdr.chunkSize = 36 + (filesize - hdr.headerSize - 4) / (hdr.dmaBlockSize + hdr.sizeOfAdditionnalDataBuffer) * hdr.numberOfChan * dataBlockSampleSize * resolutionBytes;
  whdr.subChunk2Size = whdr.chunkSize-36;
  fwrite(&whdr, sizeof(WaveHeader), 1, wavfile);

  FILE* sensorsFile = NULL;  // open mpu file
  if(argc>3){
    sensorsFile = fopen(argv[3], "w+");
    if(sensorsFile==NULL){
      printf("Failed to open sensors output file\n");
      return 0;
    }
    else
    {
      fprintf(sensorsFile,"Sensor Type,TimeStamp(ms) or Time, val0,val1,val2,val3,val4,val5,val6,val7\n");
      //val0, val1, val2 dependent du type de capteur
      //Val0 est la valeur normalisée de l'axe X pour (Accel(G), Gyr0(DPS), Mag(µT)), ou la valeur du canal1 du capteur de lumiere, ou la valeur de la temperature(°C), ou la valeur de la pression(Pa) ou le champ "fix" (pour le GPS)
      //val1 est la valeur normalisée de l'axe Y pour (Accel(G), Gyro(DPS), Mag(µT)), ou la valeur du canal2 du capteur de lumiere, ou le champ fixQuality (pour le GPS)
      //val2 est la valeur normalisée de l'axe Z pour (Accel(G), Gyro(DPS), Mag(µT)), ou le champ Latitude (pour le GPS)
    }
  }

  char* dmaBlock = (char*) malloc(hdr.dmaBlockSize);
  char* additionnalDataBlock = (char*) malloc(hdr.sizeOfAdditionnalDataBuffer);
  char* samples = (char*) malloc(hdr.numberOfChan * resolutionBytes);
  int ichan, isample;
  long pos = 0;
  bool isFirst = true;
  // read each dataBlock
  do{
    //On vient recuperer le buffer de datas additionnelles
    fread(additionnalDataBlock, hdr.sizeOfAdditionnalDataBuffer, 1, logfile);

    //On verifie le numero de version
    softwareMajorRev=additionnalDataBlock[5];
    softwareMinorRev=additionnalDataBlock[6];
    
    int enteteSize=6;
    unsigned long long timeStamp100MHzCurrentPacket=0;
    if(softwareMajorRev>=2)
    {
      //On recupere l'instant de fin du paquet courant
      timeStamp100MHzCurrentPacket=BUILD_UINT64(additionnalDataBlock[14],
                  additionnalDataBlock[13],
                  additionnalDataBlock[12],
                  additionnalDataBlock[11],
                  additionnalDataBlock[10],
                  additionnalDataBlock[9],
                  additionnalDataBlock[8],
                  additionnalDataBlock[7]);
      enteteSize=16;

      if(sensorsFile !=NULL)
      {
        //On extrait la valeur du timeStamp MHz de fin de paquet courant
        fprintf(sensorsFile,"PACKET TIMESTAMP: %ld\n",timeStamp100MHzCurrentPacket);
        //On decode les msg du buffer additionnel
        for(int i=enteteSize;i<hdr.sizeOfAdditionnalDataBuffer-enteteSize; i++)
        {
            DecodeMessage(additionnalDataBlock[i], sensorsFile);
        }
      }
    }
    else
    {
      if(sensorsFile != NULL)
      {
        parseMPU(additionnalDataBlock, hdr.sizeOfAdditionnalDataBuffer, isFirst && verbose, sensorsFile);
        isFirst = false;
      }
    }

    fread(dmaBlock, hdr.dmaBlockSize, 1, logfile);
    for(isample=0; isample<dataBlockSampleSize; isample++){
      for(ichan=0; ichan<hdr.numberOfChan; ichan++){
        memcpy(samples + ichan * resolutionBytes, dmaBlock + (ichan * dataBlockSampleSize + isample) * resolutionBytes, resolutionBytes);
      }
      fwrite(samples, resolutionBytes, hdr.numberOfChan, wavfile);
    }
    printf("\r %s : ", argv[1]);
    pos = ftell(logfile);
    printf(" %ld%%", pos*100/filesize);
  }while(pos < filesize - 1);
  printf("\r\n");
  fclose(wavfile);
  fclose(logfile);
  if(sensorsFile!=NULL){
    fclose(sensorsFile);
  }
  return 0;
}
