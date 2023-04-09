/*
 * MatFun.c
 *
 *  Created on: 2023年2月16日
 *      Author: ming
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include"MatFun.h"

void InitMatrix(Matrix *matrix, int col, int row, int value)				//初始化一个矩阵
{
	if (col>0 && row>0)
	{
		matrix->col = col;
		matrix->row = row;
		memset(matrix->data, value, sizeof(float)*col*row);
		return ;
	}
	else
		return ;
}

void ValueMatrix(Matrix *matrix, float *array) 		//给矩阵赋值
{
	if (matrix->data != NULL)
	{
		memcpy(matrix->data, array, matrix->col * matrix->row * sizeof(float));
	}
}

int SizeMatrix(Matrix *matrix)
{
	return matrix->col * matrix->row;
}

void CopyMatrix(Matrix *matrix_A, Matrix *matrix_B)
{
	matrix_B->col = matrix_A->col;
	matrix_B->row = matrix_A->row;
	memcpy(matrix_B->data, matrix_A->data, SizeMatrix(matrix_A) * sizeof(double));
}

//加法
void AddMatrix(Matrix matrix_A, Matrix matrix_B, Matrix *matrix_C)
{

	if (matrix_A.col == matrix_B.col && matrix_A.row == matrix_B.row)
	{
		for (int i = 0; i<matrix_A.row; i++)
		{
			for (int j = 0; j<matrix_A.col; j++)
			{
				matrix_C->data[i*matrix_C->col + j] = \
					matrix_A.data[i*matrix_A.col + j] + matrix_B.data[i*matrix_A.col + j];
			}
		}
	}
}


//减法
void SubMatrix(Matrix matrix_A, Matrix matrix_B, Matrix *matrix_C)
{
	matrix_C->row = matrix_A.row;
	matrix_C->col = matrix_A.col;
	if (matrix_A.col == matrix_B.col && matrix_A.row == matrix_B.row)
	{
		for (int i = 0; i<matrix_A.row; i++)
		{
			for (int j = 0; j<matrix_A.col; j++)
			{
				matrix_C->data[i*matrix_C->col + j] = \
					matrix_A.data[i*matrix_A.col + j] - matrix_B.data[i*matrix_A.col + j];
			}
		}
	}
}


//乘法
void MulMatrix(Matrix matrix_A, Matrix matrix_B, Matrix *matrix_C)
{
	matrix_C->row = matrix_A.row;

	matrix_C->col = matrix_B.col;
	memset(matrix_C->data, 0, MATSIZE*sizeof(double));
	if (matrix_A.col == matrix_B.row)		//列==行
	{

		for (int i = 0; i<matrix_A.row; i++)
		{
			for (int j = 0; j<matrix_B.col; j++)
			{
				for (int k = 0; k<matrix_A.row; k++)
				{
					matrix_C->data[i*matrix_C->col + j] += \
						matrix_A.data[i*matrix_A.col + k] * matrix_B.data[k*matrix_B.col + j];
				}
			}
		}
	}
}




//矩阵求逆S
void InvMatrix(Matrix matrix_A, Matrix *matrix_C)
{
	matrix_C->col = matrix_A.col;
	matrix_C->row = matrix_A.row;

	double f,k;
	if (matrix_A.col==2&&matrix_A.row==2)		//列==行
	{
		f = matrix_A.data[3] * matrix_A.data[0] - matrix_A.data[1] * matrix_A.data[2];
		if (f == 0) { printf("不可逆\n"); return; }
		else
			k = 1 / f;
		matrix_C->data[0] = matrix_A.data[3] *k;
		matrix_C->data[1] = -matrix_A.data[1] *k;
		matrix_C->data[2] = -matrix_A.data[2] *k;
		matrix_C->data[3] = matrix_A.data[0] * k;
	}
	else
	{
		return ;
	}
}


//矩阵转置
void TransMatrix(Matrix matrix,Matrix *matrixTemp)			//条件为方阵
{
	matrixTemp->col = matrix.row;
	matrixTemp->row = matrix.col;

	if (matrix.col>0&& matrix.row>0)
	{

		for (int i = 0; i<matrix.col; i++)
		{
			for (int j = 0; j<matrix.row; j++)
			{
				matrixTemp->data[i*matrixTemp->col + j] = matrix.data[j*matrix.col + i];
			}
		}
		return ;
	}
}
