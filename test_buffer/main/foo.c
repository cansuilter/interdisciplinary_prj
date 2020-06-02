    /* Always include the header file that declares something
                     * in the C file that defines it. This makes sure that the
                     * declaration and definition are always in-sync.  Put this
                     * header first in foo.c to ensure the header is self-contained.
                     */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

RingbufHandle_t buf_handle ;
volatile int mert;
int buffer_creator() {
	mert=13;
	//Create ring buffer
	buf_handle = xRingbufferCreate(1028, RINGBUF_TYPE_NOSPLIT);
    return mert;
}
void buffer_read(){
	//Receive an item from no-split ring buffer
	size_t item_size;
	char *item = (char *)xRingbufferReceive(buf_handle, &item_size, pdMS_TO_TICKS(1000));

	//Check received item
	if (item != NULL) {
		//Print item
		for (int i = 0; i < item_size; i++) {
			printf("%c", item[i]);
		}
		printf("\n");
		//Return Item
		vRingbufferReturnItem(buf_handle, (void *)item);
	} else {
		//Failed to receive item
	//	printf("Failed to receive item\n");
	}
}










