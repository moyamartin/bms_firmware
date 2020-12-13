#ifndef _LOGGING_H_
#define _LOGGING_H_ 

#ifdef DEBUG
	#define _DEBUG(fmt, args...) printf("%s:%s:%d: "fmt, __FILE__, __FUNCTION__, __LINE__, ##args)
#else
	#define _DEBUG(fmt, args...) {}
#endif

#endif	/* logging.h */ 
