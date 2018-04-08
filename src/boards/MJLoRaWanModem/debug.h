#ifndef __DEBUG_H__
#define __DEBUG_H__

//#define LORAWAN_DEBUG

#if defined(LORAWAN_DEBUG)
#define DEBUG_OUTPUT(fmt,arg...)	printf(fmt, ##arg)
#else
#define DEBUG_OUTPUT(fmt,arg...)
#endif

#endif