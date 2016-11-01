/* Copyright (c)  2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

 /** @cond To make doxygen skip this file */

/** @file
 *  This header contains defines with respect persistent storage that are specific to
 *  persistent storage implementation and application use case.
 */
#ifndef PSTORAGE_PL_H__
#define PSTORAGE_PL_H__

#include <stdint.h>

#define PSTORAGE_FLASH_PAGE_SIZE    ((uint16_t)NRF_FICR->CODEPAGESIZE)   /**< Size of one flash page. */
#define PSTORAGE_FLASH_EMPTY_MASK    0xFFFFFFFF                          /**< Bit mask that defines an empty address in flash. */

#define PSTORAGE_FLASH_PAGE_END     (0x39800/PSTORAGE_FLASH_PAGE_SIZE)
#define PSTORAGE_DATA_START_ADDR    ((PSTORAGE_FLASH_PAGE_END - 7) * PSTORAGE_FLASH_PAGE_SIZE)  /**< Start address for persistent data, configurable according to system requirements. */
#define PSTORAGE_DATA_END_ADDR      (PSTORAGE_FLASH_PAGE_END * PSTORAGE_FLASH_PAGE_SIZE)        /**< End address for persistent data, configurable according to system requirements. */
#define PSTORAGE_MAX_APPLICATIONS   4                                                           /**< Maximum number of applications that can be registered with the module, configurable based on system requirements. */
#define PSTORAGE_MIN_BLOCK_SIZE     0x0010                                                      /**< Minimum size of block that can be registered with the module. Should be configured based on system requirements, recommendation is not have this value to be at least size of word. */
#define PSTORAGE_MAX_BLOCK_SIZE     PSTORAGE_FLASH_PAGE_SIZE                                    /**< Maximum size of block that can be registered with the module. Should be configured based on system requirements. And should be greater than or equal to the minimum size. */
#define CMD_QUEUE_SIZE              20                                                          /**< Maximum number of flash access commands that can be maintained by the module for all applications. Configurable. */

/**@brief Handles Flash Access Result Events. To be called in the system event dispatcher of the application. */
void pstorage_sys_event_handler (uint32_t sys_evt);

#endif // PSTORAGE_PL_H__

/** @} */
/** @endcond */
