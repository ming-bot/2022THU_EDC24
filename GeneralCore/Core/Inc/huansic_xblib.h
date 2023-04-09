/*
 * huansic_xblib.h
 *
 *  Created on: Sep 29, 2022
 *      Author: Zonghuan Wu
 */

#ifndef INC_HUANSIC_XBLIB_H_
#define INC_HUANSIC_XBLIB_H_

#include "huansic_types.h"
#include "huansic_malloc.h"
#include "ming_malloc.h"

void huansic_xb_init(XB_HandleTypeDef *hxb);

enum XB_STATUS huansic_xb_decodeHeader(XB_HandleTypeDef *hxb);

enum XB_STATUS huansic_xb_decodeBody(XB_HandleTypeDef *hxb);

void huansic_xb_requestGameInfo(XB_HandleTypeDef *hxb);

void huansic_xb_setBeacon(XB_HandleTypeDef *hxb);

void ming_xb_setspeed(XB_HandleTypeDef *hxb, int16_t _speedpid);

void huansic_xb_dma_error(XB_HandleTypeDef *hxb);

void huansic_xb_it_error(XB_HandleTypeDef *hxb);

enum XB_STATUS huansic_xb_isr(XB_HandleTypeDef *hxb);

enum XB_STATUS huansic_xb_dma_isr(XB_HandleTypeDef *hxb);

#endif /* INC_HUANSIC_XBLIB_H_ */
