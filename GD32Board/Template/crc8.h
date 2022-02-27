#ifndef _CRC8_H_
#define _CRC8_H_

void crc8(unsigned char *crc, unsigned char m);
void getStrCrc8(unsigned char *crc , unsigned char *str , int length);
int check_crc8(unsigned char *data);

#endif // _CRC8_H_
