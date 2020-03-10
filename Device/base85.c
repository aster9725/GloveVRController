/*
 * base85.c
 *
 * Created: 2020-03-09 오전 9:36:41
 *  Author: YounSeok Shim
 */ 
#include "base85.h"

static uint32_t const B85P[] = { 52200625ul, 614125ul, 7225ul, 85ul, 1ul };

static char const btoascii[] = {
	'0','1','2','3','4','5','6','7','8','9',
	'A','B','C','D','E','F','G','H','I','J','K','L','M',
	'N','O','P','Q','R','S','T','U','V','W','X','Y','Z',
	'a','b','c','d','e','f','g','h','i','j','k','l','m',
	'n','o','p','q','r','s','t','u','v','w','x','y','z',
	'!','#','$','%','&','(',')','*','+','-',';',
	'<','=','>','?','@','^','_','`','{','|','}','~',
};

static void u32tob85(uint8_t* dest, uint32_t val)
{
	uint32_t bin;
	uint8_t i;
	for(i = 0;i < 5; ++i)
	{
		bin = val / B85P[i];
		dest[i] = btoascii[bin];
		val -= bin * B85P[i];
	}
}

static uint32_t u8tou32(uint8_t* src, uint8_t size)
{
	uint32_t val = 0;
	uint8_t* pVal = (uint8_t*)&val;
	uint8_t* pSrc = src;
	uint8_t i;
	
	for(i = 0; i < size; ++i)
		pVal[i] = pSrc[i];
	for(i = size; i < 4; ++i)
		pVal[i] = 0;
	
	return val;
}


void btob85(uint8_t* dest, uint8_t* src, uint8_t size)
{
	uint8_t quartets = size / 4, i;
	
	for(i = 0; i < quartets; ++i, dest+=5, src+=4)
		u32tob85(dest, u8tou32(src, 4));
	
	if(size % 4)
	{
		u32tob85(dest, u8tou32(src, size % 4));
		dest += 5;
	}
	*dest = ']';
}


static uint8_t const asciitob[] = {
	85, 62, 85, 63, 64, 65, 66, 85, 67, 68, 69, 70, 85, 71, 85, 85,
	0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 85, 72, 73, 74, 75, 76,
	77, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
	25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 85, 85, 85, 78, 79,
	80, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
	51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 81, 82, 83, 84, 85,
};

static void u32tou8(uint8_t* dest, uint32_t val)
{
	uint8_t* pDest = dest;
	uint8_t* pVal = (uint8_t*)&val;
	uint8_t i;
	
	for(i = 0; i < 4; ++i)
	{
		pDest[i] = pVal[i];
	}
}

void b85tob(uint8_t* dest, uint8_t* src)
{
	uint8_t* pSrc = src;
	uint32_t val = 0;
	uint8_t i, b;

	while(1)
	{
		val = 0;
		for(i = 0; i < 5; ++i, ++pSrc)
		{
			if(*pSrc > 128)
				return;
			b = asciitob[(*pSrc) - 32];
			if(b == 85u)
				return;
			val += b * B85P[i];
		}
		u32tou8(dest, val);
		dest += 4;
	}
	
}