/*******************************************************************************
Vendor: Xilinx 
Associated Filename: mmult1.c
Purpose: HLS C matrix multiply kernel example
Revision History: July 1, 2013 - initial release
                                                
*******************************************************************************
Copyright (C) 2013 XILINX, Inc.

This file contains confidential and proprietary information of Xilinx, Inc. and 
is protected under U.S. and international copyright and other intellectual 
property laws.

DISCLAIMER
This disclaimer is not a license and does not grant any rights to the materials 
distributed herewith. Except as otherwise provided in a valid license issued to 
you by Xilinx, and to the maximum extent permitted by applicable law: 
(1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND WITH ALL FAULTS, AND XILINX 
HEREBY DISCLAIMS ALL WARRANTIES AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, 
INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT, OR 
FITNESS FOR ANY PARTICULAR PURPOSE; and (2) Xilinx shall not be liable (whether 
in contract or tort, including negligence, or under any other theory of 
liability) for any loss or damage of any kind or nature related to, arising under 
or in connection with these materials, including for any direct, or any indirect, 
special, incidental, or consequential loss or damage (including loss of data, 
profits, goodwill, or any type of loss or damage suffered as a result of any 
action brought by a third party) even if such damage or loss was reasonably 
foreseeable or Xilinx had been advised of the possibility of the same.

CRITICAL APPLICATIONS
Xilinx products are not designed or intended to be fail-safe, or for use in any 
application requiring fail-safe performance, such as life-support or safety 
devices or systems, Class III medical devices, nuclear facilities, applications 
related to the deployment of airbags, or any other applications that could lead 
to death, personal injury, or severe property or environmental damage 
(individually and collectively, "Critical Applications"). Customer assumes the 
sole risk and liability of any use of Xilinx products in Critical Applications, 
subject only to applicable laws and regulations governing limitations on product 
liability. 

THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS PART OF THIS FILE AT 
ALL TIMES.

*******************************************************************************/
//#include <autopilot_tech.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <ap_int.h>
#include <string.h>
#include <ap_utils.h>
#include <hls_stream.h>

#define PE_NUMS 30
#define PART_TASK_NUMS 64
#define LOG2_PART_TASK_NUMS 6
//#define DATA_SIZE 102400
//#define RESULT_SIZE 24576
#define TOTAL_TASK_NUMS 1024
#define DATA_SIZE 642
#define RESULT_SIZE TOTAL_TASK_NUMS*4

//int __inc;
//#define __inc 32

void sw_extend(unsigned short qs_baddr, char *qs, unsigned short ts_baddr, short qlen, short tlen, char o_ins,
			   char e_ins, char o_del, char e_del, char penClip, char w_in, char h0, short *regScore, short qBeg, short max_ins, short max_del,
			   short *w_ret, short *qle_ret, short *tle_ret, short *gtle_ret, short *gscore_ret, short *maxoff_ret);
void proc_element(hls::stream<int>& pe_seeds, hls::stream<int>& pe_seeds_ctrl, hls::stream<int>& pe_matchs);
void write_match(int *oneMatchBuf, hls::stream<int>& pe_matchs);
void leftright_ext (int *oneSeedBuf, int *oneMatchBuf);
void load_data(int *totalinp, int *partInp, int startTaskIndex, int partTaskNums);
void load_task(int *partInp,int startTaskIndex, int partTaskNums, int staOffset, int i, int *oneTaskBuf);
bool feed_seed(int *oneTaskBuf, hls::stream<int>& pe_seeds, hls::stream<int>& pe_seeds_ctrl);
void task_parse(int *partInp, int partIndex, int partNums, int startTaskIndex, int partTaskNums, hls::stream<int> seeds[PE_NUMS], hls::stream<int> seeds_ctrl[PE_NUMS]);
void data_parse(int *totalinp, int __inc, hls::stream<int> seeds[PE_NUMS], hls::stream<int> seeds_ctrl[PE_NUMS]);
//void data_parse(int *totalinp, hls::stream<int> seeds[PE_NUMS], hls::stream<int> seeds_ctrl[PE_NUMS]);
void receive_match(hls::stream<int>& pe_matchs, int *results_buff);
void results_assemble(hls::stream<int> matchs[PE_NUMS], int *results, int __inc);
//void results_assemble(hls::stream<int> matchs[PE_NUMS], int *results);

void sw_extend(unsigned short qs_baddr, char *qs, unsigned short ts_baddr, short qlen, short tlen, char o_ins,
			   char e_ins, char o_del, char e_del, char penClip, char w_in, char h0, short *regScore, short qBeg, short max_ins, short max_del,
			   short *w_ret, short *qle_ret, short *tle_ret, short *gtle_ret, short *gscore_ret, short *maxoff_ret)
{
	ap_int<12> i;
	ap_int<10> j;
	ap_int<4> k;
	ap_int<12> max_i, max_ie, max_off;
	ap_int<12> gscore;
	char max_j;
	char oe_del = o_del + e_del;
	char oe_ins = o_ins + e_ins;
	ap_int<10> beg, end;
	char backw_tmp=0;
	char backw_reg=0;
	char forw_update=0;
	char forw_tmp=0;
	char forw_reg=0;
	ap_int<10> abs_mj_m_i;
	char tmp_ehh_m_eins;
	char tmp_eme;
	char h1_init_val;
	char max;
	char h, e;
	char e_tmp;
	char h_tmp;
	char h1_reg;
	char t, f = 0, h1, m = 0;
	char mj = -1;
	char q_i = 0, q_j = 0;
	ap_int<10> prev;
	char isBreak;
	char aw1;
	char aw_tmp;
	char h0_arr[2];
#pragma HLS ARRAY_PARTITION variable=h0_arr complete dim=0

	const char my_mat[5][5]={{1, -4, -4, -4, -1}, {-4, 1, -4, -4, -1}, {-4, -4, 1, -4, -1}, {-4, -4, -4, 1, -1}, {-1, -1, -1, -1, -1}};
#pragma HLS ARRAY_PARTITION variable=my_mat complete dim=0
	char eh_h [256];
#pragma HLS ARRAY_MAP variable=eh_h instance=eh_arr vertical
#pragma HLS RESOURCE variable=eh_h core=RAM_2P_BRAM
	char eh_e [256];
#pragma HLS ARRAY_MAP variable=eh_e instance=eh_arr vertical
#pragma HLS RESOURCE variable=eh_e core=RAM_2P_BRAM

	max = h0;
	max_i = max_j = -1;
	max_ie = -1;
	gscore = -1;
	max_off = 0;

	k = 0;
	isBreak = 0;
ext_while_loop : while ((k < 2) && (!isBreak))
				 {
#pragma HLS LOOP_TRIPCOUNT min=2 max=2
					 prev = *regScore;
					 aw_tmp = w_in << k;
					 aw1 = aw_tmp < max_ins ? aw_tmp : max_ins;
					 aw1 = aw1 < max_del ? aw1 : max_del;
					 beg = 0;
					 end = qlen;
					 if (h0 > oe_ins) {
						 tmp_eme = h0 - oe_ins;
					 }
					 else {
						 tmp_eme = 0;
					 }
					 h1_init_val = h0 - o_del;
target_loop : for (i = 0; i < tlen; i++) {
					 f = 0; m = 0; mj = -1;
					 q_i = qs[ts_baddr + i];
					 h1_init_val -= e_del;
					 h1 = h1_init_val;
					 if (h1 < 0) h1 = 0;
					 if (beg < i - aw1) beg = i - aw1;
					 if (end > i + aw1 + 1) end = i + aw1 + 1;
					 if (end > qlen) end = qlen;
					 backw_tmp = 0; backw_reg = 0;
					 forw_tmp = 0; forw_reg = 0;
					 forw_update = 0;
query_loop : for (j = beg; j < end; ++j) {
#pragma HLS LOOP_TRIPCOUNT min=50 max=50
#pragma AP pipeline II=1
#pragma AP dependence variable=eh_e array inter false
#pragma AP dependence variable=eh_h array inter false
					 q_j = qs[qs_baddr + j];
					 h_tmp = eh_h[j];// get H(i-1,j-1) and E(i-1,j)
					 e_tmp = eh_e[j];
					 if (i == 0) {
						 e = 0;
						 if (j == 0) {
							 h = h0;
						 }
						 else if (j == 1) {
							 h = tmp_eme;
						 }
						 else {
							 tmp_eme -= e_ins;
							 if (tmp_eme > 0) {
								 h = tmp_eme;
							 }
							 else {
								 h = 0;
							 }
						 }
					 }
					 else {
						 e = e_tmp;
						 h = h_tmp;
					 }
					 h1_reg = h1;
					 h += my_mat[q_i][q_j];
					 h = h > e? h : e;
					 h = h > f? h : f;
					 h1 = h;             // save H(i,j) to h1 for the next column
					 if (h1_reg == 0) {
						 backw_tmp = 0;
					 }
					 else {
						 backw_tmp++;
					 }
					 if (m <= h)
					 {
						 mj = j;
						 m = h;
						 backw_reg = backw_tmp;
					 }
					 if (j >= mj+2) {
						 if (forw_update == 0) { //((h1_reg == 0) &&
							 if (h1_reg == 0) {
								 forw_update = 1;
							 }
							 else {
								 forw_tmp++;
							 }
						 }
					 }
					 else {
						 forw_tmp = 0;
						 forw_update = 0;
					 }
					 t = h - oe_del;
					 t = t > 0? t : 0;
					 e -= e_del;
					 e = e > t? e : t;   // computed E(i+1,j)
					 eh_e[j] = e; // save E(i+1,j) for the next row
					 eh_h[j] = h1_reg;          // set H(i,j-1) for the next row
					 t = h - oe_ins;
					 t = t > 0? t : 0;
					 f -= e_ins;
					 f = f > t? f : t;   // computed F(i,j+1)
			 }
			 eh_h[end] = h1;
			 eh_e[end] = 0;
			 if ((forw_update == 0) && (h1 != 0)) {
				 if ((j >= mj+2) || (forw_tmp != 0)) {
					 forw_tmp++;
				 }
			 }
			 if (j == qlen) {
				 if (gscore <= h1) {
					 max_ie = i;
					 gscore = h1;
				 }
			 }
			 if (m == 0) break;
			 if (m > max) {
				 max = m; max_i = i; max_j = mj;
				 if (mj >= i) abs_mj_m_i = mj - i;
				 else abs_mj_m_i = i - mj;
				 if (max_off < abs_mj_m_i) max_off = abs_mj_m_i;
			 }
			 j = mj - backw_reg;
			 beg = j + 1;
			 j = mj + 2 + forw_tmp;
			 end = j;
			  }
			  *qle_ret = max_j + 1;
			  *tle_ret = max_i + 1;
			  *gtle_ret = max_ie + 1;
			  *gscore_ret = gscore;
			  *maxoff_ret = max_off;
			  *regScore = max;
			  if (max == prev || ( max_off < (aw_tmp >> 1) + (aw_tmp >> 2))) isBreak = 1;
			  k++;
			}
			*w_ret = aw_tmp;
}

void proc_element(hls::stream<int>& pe_seeds, hls::stream<int>& pe_seeds_ctrl, hls::stream<int>& pe_matchs)
{
	int i;
	short j;
	int endFlag;
	int seedLen;
	int oneSeedBuf[256];
	int matchsBuf[8];

	for (i=0; i<8192; i++) {
		while (1) {
			if (!pe_seeds_ctrl.empty()) {
				break;
			}
		}
		pe_seeds_ctrl.read(endFlag);
		if (endFlag == 0xFFFFFFFF) {
			break;
		}
		pe_seeds.read(seedLen);
		for (j=0; j<seedLen; j++) {
			pe_seeds.read(oneSeedBuf[j]);
		}
		leftright_ext(oneSeedBuf, matchsBuf);
		write_match(matchsBuf, pe_matchs);
	}
}

void write_match(int *oneMatchBuf, hls::stream<int>& pe_matchs)
{
#pragma HLS INLINE
	char i;

	for (i=0; i<5; i++) {
		pe_matchs.write(oneMatchBuf[i]);
	}
}

void leftright_ext(int *oneSeedBuf, int *oneMatchBuf)
{
//#pragma HLS INLINE
	int ii = 0;
	char o_del = 0;
	char e_del = 0;
	char o_ins = 0;
	char e_ins = 0;
	char penClip[2];
#pragma HLS ARRAY_PARTITION variable=penClip complete dim=0
	char w_in = 0;
	static int result_data = 0;
	int tmp_compar = 0;
	static int seed_index = 0;
	short qlen[2];
#pragma HLS ARRAY_PARTITION variable=qlen complete dim=0
	short tlen[2];
#pragma HLS ARRAY_PARTITION variable=tlen complete dim=0
	short max_ins[2];
#pragma HLS ARRAY_PARTITION variable=max_ins complete dim=0
	short max_del[2];
#pragma HLS ARRAY_PARTITION variable=max_del complete dim=0
	char h0;
	short regScore;
	short qBeg_ori;

	short qle;
	short tle;
	short gtle;
	short gscore;
	short maxoff;
	short qBeg;
	short rBeg;
	short qEnd;
	short rEnd;
	short score;
	short trueScore;
	short width;

	ap_uint<4> i = 0;
	short k = 0;
	ap_uint<4> l = 0;
	short qlen2 = 0;
	short tlen2 = 0;
	unsigned short qs_baddr = 0;
	unsigned short ts_baddr = 0;
	short aw[2];
#pragma HLS ARRAY_PARTITION variable=aw complete dim=0
	int tmp_parame = 0;
	int qr_offset = 0;
	short sc0 = 0;
	short h0_arr[2];
#pragma HLS ARRAY_PARTITION variable=h0_arr complete dim=0

	char query_mem[2048];
#pragma HLS RESOURCE variable=query_mem core=RAM_2P_BRAM
	int tmp_qr_data = 0;
	int param_mem[8];
	int param_data = 0;
	int query_data = 0;
	int ctrl_flag = 0;
	int qrLen_div8 = 0;


//	for (ii=0; ii<8196; ii++)
	{
//		pe_seeds.read(seed_index);
//		if (seed_index == 0xFFFFFFFF) {
//			break;
//		}
		seed_index = oneSeedBuf[0];
//		pe_seeds.read(tmp_compar);
		tmp_compar = oneSeedBuf[1];
		o_del = tmp_compar & 0xFF; //6
		e_del = (tmp_compar >> 8) & 0xFF; //1
		o_ins = (tmp_compar >> 16) & 0xFF; //6
		e_ins = (tmp_compar >> 24) & 0xFF; //1
//		pe_seeds.read(tmp_compar);
		tmp_compar = oneSeedBuf[2];
		penClip[0] = tmp_compar & 0xFF; //100
		penClip[1] = (tmp_compar >> 8) & 0xFF;
		w_in = (tmp_compar >> 16) & 0xFF;

	load_param :
		for (k = 0; k < 8; k++) {
//			pe_seeds.read(param_data);
//			param_mem[k] = param_data;
			param_mem[k] = oneSeedBuf[3+k];
		}

		 tmp_parame = param_mem[0];
		 qlen[0] = tmp_parame & 0xFFFF; //55
		 tlen[0] = (tmp_parame >> 16) & 0xFFFF; //105
		 if (qlen[0] > 200) {
			 qlen[0] = 200;
		 }
		 if (tlen[0] > 2000) {
			 tlen[0] = 2000;
		 }
		 tmp_parame = param_mem[1];
		 qlen[1] = tmp_parame & 0xFFFF; //55
		 tlen[1] = (tmp_parame >> 16) & 0xFFFF; //105
		 if (qlen[1] > 200) {
			 qlen[1] = 200;
		 }
		 if (tlen[1] > 2000) {
			 tlen[1] = 2000;
		 }
		 tmp_parame = param_mem[2];
		 qr_offset = tmp_parame;
		 tmp_parame = param_mem[3];
		 regScore = tmp_parame & 0xFFFF; //100
		 qBeg_ori = (tmp_parame >> 16) & 0xFFFF;
		 tmp_parame = param_mem[4];
		 h0 = tmp_parame & 0xFFFF;
		 tmp_parame = param_mem[5];
		 max_ins[0] = tmp_parame & 0xFFFF;
		 max_del[0] = (tmp_parame >> 16) & 0xFFFF;
		 tmp_parame = param_mem[6];
		 max_ins[1] = tmp_parame & 0xFFFF;
		 max_del[1] = (tmp_parame >> 16) & 0xFFFF;

		 aw[0] = w_in;
		 aw[1] = w_in;
		 qBeg = 0;
		 qEnd = qlen[1];
		 rBeg = 0;
		 rEnd = 0;
		 trueScore = regScore;
		 qle = -1;
		 tle = -1;
		 gtle = -1;
		 gscore = -1;
		 maxoff = -1;
		 qlen2 = qlen[0] + qlen[1];
		 tlen2 = tlen[0] + tlen[1];
		 qrLen_div8 = qlen2+tlen2;
		 if ((qrLen_div8 & 0x00000007) != 0) {
			 qrLen_div8 = (qrLen_div8 >> 3) + 1;
		 }
		 else {
			 qrLen_div8 = (qrLen_div8 >> 3);
		 }
load_query :
		 for (k=0; k<qrLen_div8; k++) {
//			 pe_seeds.read(tmp_qr_data);
			 tmp_qr_data = oneSeedBuf[11+k];
			 for (l=0; l<8; l++) {
				 query_mem[k*8 + l] = (tmp_qr_data & 0xF0000000) >> 28;
				 tmp_qr_data <<= 4;
			 }
		 }
		 qs_baddr = 0;
		 ts_baddr = qlen2;
left_right_loop :
		 for (i=0; i<2; i++)
		  {
#pragma HLS LOOP_TRIPCOUNT min=2 max=2
			  sc0 = regScore;
			  h0_arr[0] = h0;
			  h0_arr[1] = sc0;
			  if (qlen[i] > 0) {
				  sw_extend(qs_baddr, query_mem, ts_baddr, qlen[i], tlen[i], o_ins, e_ins, o_del, e_del, penClip[i],
					  w_in, h0_arr[i], &regScore, qBeg_ori, max_ins[i], max_del[i], &aw[i], &qle, &tle, &gtle, &gscore, &maxoff);
				  score = regScore;
				  if (gscore <= 0 || gscore <= (regScore - penClip[i])) {
					  if (i == 0) {
						  qBeg = qBeg_ori - qle;
						  rBeg = -tle;
						  trueScore = regScore;
					  }
					  else {
						  qEnd = qle;
						  rEnd = tle;
						  trueScore += regScore - sc0;
					  }
				  }
				  else {
					  if (i == 0) {
						  qBeg = 0;
						  rBeg = -gtle;
						  trueScore = gscore;
					  }
					  else {
						  qEnd = qlen[1];
						  rEnd = gtle;
						  trueScore += gscore - sc0;
					  }
				  }
			  }
			  qs_baddr += qlen[i];
			  ts_baddr += tlen[i];
		  }
		  if (aw[0] > aw[1]) width = aw[0];
		  else width = aw[1];

//		  pe_matchs.write(seed_index);
		  oneMatchBuf[0] = (seed_index);
		  result_data = (qBeg & 0xFFFF) | ((qEnd<<16) & 0xFFFF0000);
//		  pe_matchs.write(result_data);
		  oneMatchBuf[1] = (result_data);
		  result_data = (rBeg & 0xFFFF) | ((rEnd<<16) & 0xFFFF0000);
//		  pe_matchs.write(result_data);
		  oneMatchBuf[2] = (result_data);
		  result_data = (score & 0xFFFF) | ((trueScore<<16) & 0xFFFF0000);
//		  pe_matchs.write(result_data);
		  oneMatchBuf[3] = (result_data);
		  result_data = width & 0xFFFF;
//		  pe_matchs.write(result_data);
		  oneMatchBuf[4] = (result_data);
//		  pe_seeds_ctrl.read();
	}
}

void load_data(int *totalinp, int *partInp, int startTaskIndex, int partTaskNums)
{
	int i=0;
	int staOffset = 0;
	int endOffset = 0;
	int firstTaskPos = 0;
	int lastTaskPos = 0;
	short qlen[2];
#pragma HLS ARRAY_PARTITION variable=qlen complete dim=0
	short tlen[2];
#pragma HLS ARRAY_PARTITION variable=tlen complete dim=0
	short qlen2 = 0;
	short tlen2 = 0;
	int dataLen = 0;
	int tmp_parame = 0;
	int qrLen_div8 = 0;

	memcpy(partInp, (const void*)(&totalinp[0]), (8)*4);
	memcpy(&partInp[8], (const void*)(&totalinp[(startTaskIndex + 1)*8]), (partTaskNums*8)*4);
	//for (i=0; i<8; i++) {
	//	partInp[i] = totalinp[i];
	//}
	//for (i=0; i<(partTaskNums*8); i++) {
	//	partInp[8+i] = totalinp[(startTaskIndex + 1)*8 + i];
	//}

	firstTaskPos = (startTaskIndex + 1)*8;
	staOffset = totalinp[firstTaskPos + 2];
	lastTaskPos = (startTaskIndex + partTaskNums)*8;
	tmp_parame = totalinp[lastTaskPos + 0];
	qlen[0] = tmp_parame & 0xFFFF; //55
	tlen[0] = (tmp_parame >> 16) & 0xFFFF; //105
	tmp_parame = totalinp[lastTaskPos + 1];
	qlen[1] = tmp_parame & 0xFFFF; //55
	tlen[1] = (tmp_parame >> 16) & 0xFFFF; //105
	endOffset = totalinp[lastTaskPos + 2];
	qlen2 = qlen[0] + qlen[1];
	tlen2 = tlen[0] + tlen[1];
	qrLen_div8 = qlen2+tlen2;
	if ((qrLen_div8 & 0x00000007) != 0) {
		qrLen_div8 = (qrLen_div8 >> 3) + 1;
	}
	else {
		qrLen_div8 = (qrLen_div8 >> 3);
	}
	//for (i=0; i<(endOffset - staOffset + qrLen_div8); i++) {
	//	partInp[(partTaskNums+1)*8 + i] = totalinp[staOffset + i];
	//}
	memcpy(&partInp[(partTaskNums+1)*8], (const void*)(&totalinp[staOffset]), (endOffset - staOffset + qrLen_div8)*4);
}
//bool feed_seed(int *partInp,int startTaskIndex, int partTaskNums, int staOffset, int i, hls::stream<int>& pe_seeds, hls::stream<int>& pe_seeds_ctrl)
void load_task(int *partInp,int startTaskIndex, int partTaskNums, int staOffset, int i, int *oneTaskBuf)
{
//#pragma HLS INLINE
	int k = 0;
	int l = 0;
	short qlen[2];
#pragma HLS ARRAY_PARTITION variable=qlen complete dim=0
	short tlen[2];
#pragma HLS ARRAY_PARTITION variable=tlen complete dim=0
	short qlen2 = 0;
	short tlen2 = 0;
	int tmp_parame = 0;
	int endOffset = 0;
	int qrLen_div8 = 0;
	int taskParaPos = 0;
	int taskDataPos = 0;

//	if (!pe_seeds_ctrl.full()) {
		taskParaPos = (i+1)*8;
		tmp_parame = partInp[taskParaPos + 0];
		qlen[0] = tmp_parame & 0xFFFF; //55
		tlen[0] = (tmp_parame >> 16) & 0xFFFF; //105
		tmp_parame = partInp[taskParaPos + 1];
		qlen[1] = tmp_parame & 0xFFFF; //55
		tlen[1] = (tmp_parame >> 16) & 0xFFFF; //105
		tmp_parame = partInp[taskParaPos + 2];
		endOffset = tmp_parame;
		qlen2 = qlen[0] + qlen[1];
		tlen2 = tlen[0] + tlen[1];
		qrLen_div8 = qlen2+tlen2;
		if ((qrLen_div8 & 0x00000007) != 0) {
			qrLen_div8 = (qrLen_div8 >> 3) + 1;
		}
		else {
			qrLen_div8 = (qrLen_div8 >> 3);
		}
		taskDataPos = endOffset - staOffset + (partTaskNums+1)*8;
//		pe_seeds.write(startTaskIndex + i);
//		pe_seeds.write(partInp[0]);
//		pe_seeds.write(partInp[1]);
		oneTaskBuf[0] = (qrLen_div8 + 11);
		oneTaskBuf[1] = (startTaskIndex + i);
		oneTaskBuf[2] = (partInp[0]);
		oneTaskBuf[3] = (partInp[1]);
		for (k = 0; k < 8; k++) {
	#pragma HLS PIPELINE II=1
//			pe_seeds.write(partInp[taskParaPos + k]);
			oneTaskBuf[4+k] = (partInp[taskParaPos + k]);
		}
		for (l = 0; l < qrLen_div8; l++) {
	#pragma HLS PIPELINE II=1
//			pe_seeds.write(partInp[taskDataPos + l]);
			oneTaskBuf[12+l] = (partInp[taskDataPos + l]);
		}
//		pe_seeds_ctrl.write(1);
//		return true;
//	}
//	else {
//		return false;
//	}

}

bool feed_seed(int *oneTaskBuf, hls::stream<int>& pe_seeds, hls::stream<int>& pe_seeds_ctrl)
{
#pragma HLS INLINE
	short i;
	int taskLen;

	if (!pe_seeds_ctrl.full()) {
		taskLen = oneTaskBuf[0];
		for (i=0; i<taskLen+1; i++) {
			pe_seeds.write(oneTaskBuf[i]);
		}
		pe_seeds_ctrl.write(1);
		return true;
	}
	else {
		return false;
	}
}

void task_parse(int *partInp, int partIndex, int partNums, int startTaskIndex, int partTaskNums, hls::stream<int> seeds[PE_NUMS], hls::stream<int> seeds_ctrl[PE_NUMS])
{
#pragma HLS INLINE
	int i = 0;
	char j = 0;
	char beg = 0;
	char m = 0;
	bool pe_avail;
	int endWords = 0;
	int staOffset = 0;
	static char avail_j = 0;
	int oneTaskBuf[256];

	endWords = 0xFFFFFFFF;
	staOffset = partInp[1*8 + 2];
	beg = 0;
	j = 0;
	for(i=0; i<partTaskNums; i++) {
		load_task(partInp, startTaskIndex, partTaskNums, staOffset, i, oneTaskBuf);
		while(1) {
			pe_avail = 0;
//			for(j=beg; j<PE_NUMS; j++) {
				if (feed_seed(oneTaskBuf, seeds[j], seeds_ctrl[j])) {
				//if (!seeds_ctrl[j].full()) {
					//feed_seed(partInp, startTaskIndex, partTaskNums, staOffset, i, seeds[j], seeds_ctrl[j]);
					//pe_avail = 1;
					//avail_j = j;
					break;
				}
				else {
					if (j >= PE_NUMS) {
						j = 0;
					}
					else {
						j = j+1;
					}
				}
//			}
//			if (pe_avail == 1) {
//				break;
//			}
		}
//		if (avail_j >= PE_NUMS-1) {
//			beg = 0;
//		}
//		else {
//			beg = avail_j + 1;
//		}
	}
	if (partIndex == (partNums)) {
		for(m=0; m<PE_NUMS; m++) {
			seeds_ctrl[m].write(endWords);
		}
	}
}

void data_parse(int *totalinp, int __inc, hls::stream<int> seeds[PE_NUMS], hls::stream<int> seeds_ctrl[PE_NUMS])
//void data_parse(int *totalinp, hls::stream<int> seeds[PE_NUMS], hls::stream<int> seeds_ctrl[PE_NUMS])
{
//#pragma HLS INLINE
	int i;
	static int partNums = 0;
	int locDataMem_a[6144];
	int locDataMem_b[6144];
	static int startTaskIndex = 0;
	static int partTaskNums = 0;
	static int startTaskIndex_reg = 0;
	static int partTaskNums_reg = 0;

	if (__inc & (PART_TASK_NUMS-1)) {
		partNums = (__inc >> LOG2_PART_TASK_NUMS) + 1;
	}
	else {
		partNums = (__inc >> LOG2_PART_TASK_NUMS);
	}
	startTaskIndex = 0;
data_parse_loop : for (i=0; i<=partNums; i++) {
	if (i == (partNums - 1)) {
		if ((__inc & (PART_TASK_NUMS-1)) == 0) {
			partTaskNums = PART_TASK_NUMS;
		}
		else {
			partTaskNums = __inc & (PART_TASK_NUMS-1);
		}
	}
	else {
		partTaskNums = PART_TASK_NUMS;
	}
	if (i%2 == 0) {
		if (i < partNums) {
			load_data(totalinp, locDataMem_a, startTaskIndex, partTaskNums);
		}
		if (i > 0) {
			task_parse(locDataMem_b, i, partNums, startTaskIndex_reg, partTaskNums_reg, seeds, seeds_ctrl);
		}
	}
	else {
		if (i < partNums) {
			load_data(totalinp, locDataMem_b, startTaskIndex, partTaskNums);
		}
		if (i > 0) {
			task_parse(locDataMem_a, i, partNums, startTaskIndex_reg, partTaskNums_reg, seeds, seeds_ctrl);
		}
	}
	startTaskIndex_reg = startTaskIndex;
	partTaskNums_reg = partTaskNums;
	startTaskIndex += partTaskNums;
	}
}

//bool receive_match(hls::stream<int>& pe_matchs, int *results_buff)
void receive_match(hls::stream<int>& pe_matchs, int *results_buff)
{
#pragma HLS INLINE
	char k;
	static int seed_index = 0;

//	if (!pe_matchs.empty()) {
		pe_matchs.read(seed_index);
		for (k = 0; k < 4; k++) {
			pe_matchs.read(results_buff[seed_index*4 + k]);
		}
//		return true;
//	}
//	else {
//		return false;
//	}

}
void results_assemble(hls::stream<int> matchs[PE_NUMS], int *results, int __inc)
//void results_assemble(hls::stream<int> matchs[PE_NUMS], int *results)
{
	int i;
	static char j;
	static char beg = 0;
	static char avail_j = 0;
	int idle_cnt = 0;
	bool err_flag;
	bool pe_done;

	for(i=0; i<__inc; i++) {
		while(1) {
			pe_done = 0;
			for(j=beg; j<PE_NUMS; j++) {
				idle_cnt += 1;
//				if (receive_match(matchs[j], results)) {
				if (!matchs[j].empty()) {
					receive_match(matchs[j], results);
					avail_j = j;
					pe_done = 1;
					idle_cnt = 0;
					break;
				}
				else {
					avail_j = j + 1;
				}
			}
			err_flag = 0;
			if (avail_j >= PE_NUMS) {
				beg = 0;
			}
			else {
				beg = avail_j;
			}
			if (pe_done == 1) {
				break;
			}
			//else if (idle_cnt > 5000) {
			//	idle_cnt = 0;
			//	err_flag = 1;
			//	break;
			//}
		}
		//if (err_flag == 1) {
		//	break;
		//}
	}
}

extern "C" {
void mmult(int *a, int *output, int __inc)
{
#pragma HLS INTERFACE m_axi port=a offset=slave bundle=gmem depth=51200
#pragma HLS INTERFACE m_axi port=output offset=slave bundle=gmem depth = 4096
#pragma HLS INTERFACE s_axilite port=a bundle=control
#pragma HLS INTERFACE s_axilite port=output bundle=control
#pragma HLS INTERFACE s_axilite port=__inc bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control

	//int total_in[DATA_SIZE];
	int resul_out[RESULT_SIZE];
	int local_inc;

	//memcpy(total_in, (int *) a, DATA_SIZE*4);
	local_inc = __inc;

//	{
#pragma HLS DATAFLOW
		int i;
		hls::stream<int> seeds[PE_NUMS];
	#pragma HLS STREAM variable=seeds depth=512
		hls::stream<int> seeds_ctrl[PE_NUMS];
	#pragma HLS STREAM variable=seeds_ctrl depth=1
		hls::stream<int> matchs[PE_NUMS];
	#pragma HLS STREAM variable=matchs depth=256

		data_parse(a, local_inc, seeds, seeds_ctrl);
		//data_parse(total_in, seeds, seeds_ctrl);
E_dupl_loop : for (i=0; i<PE_NUMS; i++) {
#pragma HLS UNROLL
			proc_element(seeds[i], seeds_ctrl[i], matchs[i]);
//	leftright_ext(seeds[i], seeds_ctrl[i], matchs[i]);
		}
        results_assemble(matchs, resul_out, local_inc);
		//results_assemble(matchs, resul_out);
//	}
   memcpy((int *) output, resul_out, RESULT_SIZE*4);
   return;
}
}

