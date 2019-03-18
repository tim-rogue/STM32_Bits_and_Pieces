/*
 * uart_debug.h
 *
 *  Created on: Jul 24, 2018
 *      Author: Andrew
 */

#ifndef UART_DEBUG_H_
#define UART_DEBUG_H_

#include <stdlib.h>
#include <string.h> /*memcpy*/
#include "stm32f7xx_hal.h"

#define UART_DEBUG_HEXDUMP_BLOCK_SIZE 16 /*bytes per dump line */
//#define UART_DEBUG_OSX 1 /*lets compiler know if debug should terminate with \r\n for UART_DEBUG_OSX formating */

int8_t UartDebug_init(UART_HandleTypeDef*);
void UartDebug_deinit(void);
uint32_t UartDebug_addToBuffer(uint8_t*, uint32_t);
void UartDebug_hexdump(uint8_t*, uint32_t);
void UartDebug_hexprint8(uint8_t val);
void UartDebug_hexprint16(uint16_t);
void UartDebug_hexprint32(uint32_t);
void UartDebug_hexprint64(uint64_t val);
void UartDebug_printuint8(uint8_t);
void UartDebug_printuint32(uint32_t);
void UartDebug_printuint64(uint64_t);
void UartDebug_putchar(const unsigned char);
void UartDebug_callback(void);
void UartDebug_newline(void);
void UartDebug_dumpbyte(uint8_t);
void UartDebug_sendline(char*);
void UartDebug_sendstring(char* str);
void UartDebug_printBool(uint32_t);
void UartDebug_memDump(uint8_t*, size_t);

void uart_debug_tx_complete_callback(DMA_HandleTypeDef *_hdma);


#endif /* UART_DEBUG_H_ */
