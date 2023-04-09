/*
 * MatFun.h
 *
 *  Created on: 2023年2月16日
 *      Author: Administrator
 */

#ifndef INC_MATFUN_H_
#define INC_MATFUN_H_

#pragma once
//申请最大矩阵内存
#define MATSIZE 20
//结构体
 typedef struct Matrix_t
{
	uint8_t row, col;		//row为行,col为列
	float data[MATSIZE];
} Matrix;

void InitMatrix(Matrix *matrix, int col, int row, int value);		//初始化矩阵
void ValueMatrix(Matrix *matrix, float *array);				//给一个矩阵赋值
int SizeMatrix(Matrix *matrix);								//获得一个矩阵的大小
void CopyMatrix(Matrix *matrix_A, Matrix *matrix_B);		//复制一个矩阵的值
void PrintMatrix(Matrix *matrix);							//打印一个矩阵

															//矩阵的基本运算
void AddMatrix(Matrix matrix_A, Matrix matrix_B, Matrix *matrix_C);		//矩阵的加法
void SubMatrix(Matrix matrix_A, Matrix matrix_B, Matrix *matrix_C);		//矩阵剑法
void MulMatrix(Matrix matrix_A, Matrix matrix_B, Matrix *matrix_C);		//矩阵的乘法
void TransMatrix(Matrix matrix, Matrix *matrix_C);							//矩阵的转置
void InvMatrix(Matrix matrix_A, Matrix *matrix_C);                         //逆矩阵


#endif /* INC_MATFUN_H_ */
