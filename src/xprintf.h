/*------------------------------------------------------------------------*/
/* Universal string handler for user console interface  (C)ChaN, 2011     */
/*------------------------------------------------------------------------*/

#ifndef __XPRINTF_H__
#define __XPRINTF_H__

#define	_CR_CRLF		1	/* 1: Convert \n ==> \r\n in the output char */


#define xdev_out(func) xfunc_out = (void(*)(char))(func)
extern void (*xfunc_out)(char);
void xputc (char c);
void xputs (const char* str);
void xfputs (void (*func)(unsigned char), const char* str);
void xprintf (const char* fmt, ...);
void xsprintf (char* buff, const char* fmt, ...);


#endif /* __XPRINTF_H__ */
