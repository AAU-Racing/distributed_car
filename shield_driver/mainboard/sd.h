#include <stdint.h>

#define SD_STANDARD_BLOCK_SIZE 512

void create_log(char *file_name);
void create_error(char *file_name);
void create_file(char *file_name, char *prefix, char *extension);
void writeTime(char *file_name, uint32_t time);
void writeStr(char *file_name, char *str, uint8_t len);
void appendData(char *file_name, void *data, uint16_t len);
