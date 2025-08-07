#ifndef __FSIA6B_H
#define __FSIA6B_H
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct _FSiA6B_iBus
{
	unsigned short RH;
	unsigned short RV;
	unsigned short LV;
	unsigned short LH;
	unsigned short SwA;
	unsigned short SwB;
	unsigned short SwC;
	unsigned short SwD;
	unsigned short VrA;
	unsigned short VrB;

	unsigned short FailSafe;
}FSiA6B_iBus;

extern FSiA6B_iBus iBus;

unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len);
void iBus_Parsing(unsigned char* data, FSiA6B_iBus* iBus);
void FSiA6B_UART4_Initialization(void);
unsigned char iBus_isActiveFailsafe(FSiA6B_iBus* iBus);

#ifdef __cplusplus
}
#endif
#endif /* __FSIA6B_H */
