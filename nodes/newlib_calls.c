#include <stdint.h>
#include <sys/stat.h>
#include <board_driver/init.h>
#include <board_driver/interrupts.h>
#include <board_driver/uart.h>
#include <board_driver/usb/usb.h>

//#define UART_AUTO_ECHO
#define STACK_POINTER (__get_MSP())

/* Using the USB interface with newlib calls
 * WARNING: Some messages might get dropped, especially in the beginning og the
 * transmission. The reason for this has not yet been found.
 * The USBD driver returns USBD_OK even when a message doesn't get through.
 */
//#define USB_SERIAL_INTERFACE

/* Symbol from linkerscript */
extern uint32_t _start_addr;


void _init(void) {
	init_board();
	set_system_clock_168mhz();
	set_interrupt_offset((uint32_t)&_start_addr);
	enable_all_interrupts();
}

void _exit(int exitcode) {
	(void)exitcode;
	while(1);
}

#ifdef USB_SERIAL_INTERFACE
int _write(int file, char *ptr, int len) {
	(void)file;

	HAL_Delay(1);
	usb_send((void *)ptr, (size_t)len);

	return len;
}
#else

int _write(int file, char *ptr, int len) {
	(void)file;

	int i;
	for (i = 0; i < len; i++) {
		if (*ptr == '\n') {
			uart_send_byte('\r');
		}
		uart_send_byte(*ptr);
		++ptr;
	}

	return i;
}
#endif

/* When newlib calls this function len is allways 1024,
 * Therefore we have to decide when to return */
#ifdef USB_SERIAL_INTERFACE
/* IMPORTANT: The usb dependent _read function below expects \n as a
terminalting charater as is UNIX/MacOS X convention. Setup the terminal emulator
correctly and you won't have problem. Some terminal emulators use \r or \r\n by
default, where \r\n is the DOS standard.

We could support \r as a terminating charater as well. But have not done so to
avoid any confusion.
*/
int _read(int file, char *ptr, int len) {
	(void)file;

	int i;
	for (i = 0; i < len; i++) {
		ptr[i] = usb_recieve();

		/* Return partial buffer if we get EOL */
		if (ptr[i] == '\n') {
			i++;
			return i;
		}
	}

	/* We make sure the buffer contains \n before we return,
	 * in case we recieve the maximum 1024 characters */
	ptr[i] = '\n';
	return i;
}
#else

int _read(int file, char *ptr, int len) {
	(void)file;

	int i;
	for (i = 0; i < len; i++) {
		ptr[i] = uart_read_byte();
#ifdef UART_AUTO_ECHO
		uart_send_byte(ptr[i]);
#endif
		/* Return partial buffer if we get EOL */
		if (ptr[i] == '\r') {
			i++;
			ptr[i] = '\n';
			return i;
		}
	}

	/* We make sure the buffer contains \r and \n before we return,
	 * in case we recieve the maximum 1024 characters */
	ptr[i-1] = '\r';
	ptr[i] = '\n';
	return i;
}
#endif

caddr_t _sbrk(int incr) {
	extern char _end;		/* Defined by the linker */
	static char *heap_end;
	char *prev_heap_end;

	if (heap_end == 0) {
		heap_end = &_end;
	}
	prev_heap_end = heap_end;
	if ((unsigned)(heap_end + incr) > STACK_POINTER) {
		/* Heap and stack collision */
		return (caddr_t)0;
	}

	heap_end += incr;
	return (caddr_t) prev_heap_end;
}

int _close(int file) {
	(void)file;
	return -1;
}

int _fstat(int file, struct stat *st) {
	(void)file;
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file) {
	switch(file) {
	case 1: // stdio
	case 2: // stderr
	return 1;

	default: return 0;
	}
}

int _lseek(int file, int ptr, int dir) {
	(void)file;
	(void)ptr;
	(void)dir;
	return 0;
}

int _open(const char *name, int flags, int mode) {
	(void)name;
	(void)flags;
	(void)mode;
	return -1;
}
