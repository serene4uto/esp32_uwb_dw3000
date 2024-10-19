/**
 * @file    tag_list.h
 *
 * @brief
 *
 * @author Decawave
 *
 * @attention Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __INC_TAG_LIST_H__
#define __INC_TAG_LIST_H__ 1

#ifdef __cplusplus
 extern "C" {
#endif

//#include <stdint.h>

//maximum is limited by NUM_SLOTS; FCONFIG_SIZE and available memory size, see default_config.h
#define MAX_KNOWN_TAG_LIST_SIZE             (20)
#define MAX_DISCOVERED_TAG_LIST_SIZE        (20)

/* Tag list */

typedef struct
{
    uint16_t    slot;
    
    union {
        uint8_t     addrShort[2];
        uint16_t    addr16;
    };

    union    {
        uint8_t      addrLong[8];
        uint64_t     addr64;
    };

}__attribute__((packed))
tag_addr_slot_t;


#ifdef __cplusplus
}
#endif

#endif /* __INC_TAG_LIST_H_ */
