#ifndef _PERIPHAL_H_
#define _PERIPHAL_H_

#include "global.h"

typedef enum {
	STATUS_OK, 			/**< success, no error occurred                          */
	STATUS_ERROR, 		/**< failed, some unspecified error occured              */
	STATUS_TIMEOUT, 	/**< failed, timeout occurred                            */
	STATUS_UNAVAILABLE,	/**< failed, function unavailable                        */
	STATUS_WARNING 		/**< success, but the result is probably not as expected */
} ExitStatus_t;

#endif /* _PERIPHAL_H_ */

