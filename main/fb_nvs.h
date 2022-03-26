/*
    ...

*/

#ifndef __FB_NVS_H__
#define __FB_NVS_H__

bool set_flash_uint32( uint32_t ip, const char *label );

bool get_flash_uint32( uint32_t *ip, const char *label );

bool set_flash_uint8( uint8_t dataToStore, const char *label );

bool get_flash_uint8( uint8_t *pDataToRead, const char *label );

void init_nvs_data();

void nvs_task(void* arg);

#endif /* __FB_NVS_H__ */