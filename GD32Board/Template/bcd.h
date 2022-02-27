#ifndef _BCD_H
#define _BCD_H

#define BCD2BIN(x)	(((x) & 0x0f) + ((x) >> 4) * 10)
#define BIN2BCD(x)	((((x) / 10) << 4) + (x) % 10)

#endif
