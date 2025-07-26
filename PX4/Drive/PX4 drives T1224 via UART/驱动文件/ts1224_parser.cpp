/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file modified from sf0x_parser.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Chuong Nguyen <chnguye7@asu.edu>
 * @author Ayush Gaud <ayush.gaud@gmail.com>
 *
 * Declarations of parser for the Benewake TFmini laser rangefinder series
 */

#include "ts1224_parser.h"
#include <string.h>
#include <stdlib.h>


int TS1224_parse(char c, char *parserbuf, unsigned *parserbuf_index, TS1224_PARSE_STATE *state, float *dist)
{
	int ret = -1;
	//char *end;

	switch (*state) {
	case TS1224_PARSE_STATE::STATE0_MsgType:
		if (c == 0xFB) {
			*state = TS1224_PARSE_STATE::STATE1_MsgCode;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = TS1224_PARSE_STATE::STATE0_MsgType;
		}

		break;

	case TS1224_PARSE_STATE::STATE1_MsgCode:
		if (c == 0x03) {
			*state = TS1224_PARSE_STATE::STATE2_BrdId;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = TS1224_PARSE_STATE::STATE0_MsgType;
		}

		break;

	case TS1224_PARSE_STATE::STATE2_BrdId:
		*state = TS1224_PARSE_STATE::STATE3_PayLoadLen;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case TS1224_PARSE_STATE::STATE3_PayLoadLen:
		if (c == 0x04) {
			*state = TS1224_PARSE_STATE::STATE4_DataValidInd_L;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;
		}else{
			*state = TS1224_PARSE_STATE::STATE0_MsgType;
		}

		break;

	case TS1224_PARSE_STATE::STATE4_DataValidInd_L:
		*state = TS1224_PARSE_STATE::STATE5_DataValidInd_H;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case TS1224_PARSE_STATE::STATE5_DataValidInd_H:
		*state = TS1224_PARSE_STATE::STATE6_Distance_L;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case TS1224_PARSE_STATE::STATE6_Distance_L:
		*state = TS1224_PARSE_STATE::STATE7_Distance_H;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case TS1224_PARSE_STATE::STATE7_Distance_H:
		*state = TS1224_PARSE_STATE::STATE8_CRC;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case TS1224_PARSE_STATE::STATE8_CRC:
		// Find the checksum
		unsigned short cksm = 0;

		for (int i = 0; i < 8; i++) {
			cksm += parserbuf[i];
		}

		if (c == (cksm & 0x00FF)) {
			parserbuf[*parserbuf_index] = '\0';
			unsigned int t1 = parserbuf[6];
			unsigned int t2 = parserbuf[7];
			t2 <<= 8;
			t2 += t1;

			if (t2 < 0xFFFFu) {
				*dist = ((float)t2) / 10;
			}

			*state = TS1224_PARSE_STATE::STATE0_MsgType;
			*parserbuf_index = 0;
			ret = 0;

		} else {
			*state = TS1224_PARSE_STATE::STATE0_MsgType;
			*parserbuf_index = 0;
		}

		break;
	}

	return ret;
}
