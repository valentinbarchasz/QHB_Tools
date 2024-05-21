
#define BUILD_UINT16(loByte, hiByte) \
          ((unsigned short)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))
#define BUILD_INT16(hiByte, loByte) \
          ((int)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))          
#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((unsigned long)((unsigned long)((Byte0) & 0x00FF) \
          + ((unsigned long)((Byte1) & 0x00FF) << 8) \
          + ((unsigned long)((Byte2) & 0x00FF) << 16) \
          + ((unsigned long)((Byte3) & 0x00FF) << 24)))

#define BUILD_UINT64(Byte0, Byte1, Byte2, Byte3,Byte4, Byte5, Byte6, Byte7) \
          ((unsigned long long)((unsigned long long)((Byte0) & 0x00FF) \
          + ((unsigned long long)((Byte1) & 0x00FF) << 8) \
          + ((unsigned long long)((Byte2) & 0x00FF) << 16) \
          + ((unsigned long long)((Byte3) & 0x00FF) << 24)\
          + ((unsigned long long)((Byte4) & 0x00FF) << 32)\
          + ((unsigned long long)((Byte5) & 0x00FF) << 40)\
          + ((unsigned long long)((Byte6) & 0x00FF) << 48)\
          + ((unsigned long long)((Byte7) & 0x00FF) << 56)))
          