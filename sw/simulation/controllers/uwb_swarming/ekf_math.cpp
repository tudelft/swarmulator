/**
 * @file ekf_math.cpp
 * @author Sven Pfeiffer, MAVLab, TU Delft
 * @date 23 Apr 2024
 * @brief Definitions of matrix operations needed in the ekf
 * 
 * This file contains functions for common matrix operations 
 * that are required by the relative localization ekf or the 
 * AgentInitializer
 */
#include "ekf_math.h"

void fmat_set_zero(MatrixFloat &A){
    for (unsigned int i=0; i<A.rows; i++){
        for (unsigned int j=0; j<A.cols; j++){
            A.data[i][j] = 0;
        }
    }
}

void fmat_set_identity(MatrixFloat &A){
    if (A.cols != A.rows){
        throw std::invalid_argument("fmat_set_identity: Matrix must be square");
    }
    for (unsigned int i=0; i<A.rows; i++){
        for (unsigned int j=0; j<A.cols; j++){
            A.data[i][j] = 0;
        }
        A.data[i][i] = 1;
    }
}

void fmat_trans(const MatrixFloat &A, MatrixFloat &AT){
    if (A.cols != AT.rows || A.rows != AT.cols){
        throw std::invalid_argument("fmat_trans: Matrix dimensions do not match");
    }

    for (unsigned int i=0; i<A.rows; i++){
        for (unsigned int j=0; j<A.cols; j++){
            AT.data[j][i] = A.data[i][j];
        }
    }
}

void fmat_mult(const MatrixFloat &A, const MatrixFloat &B, MatrixFloat &R){
    if (A.cols != B.rows || A.rows != R.rows || B.cols != R.cols){
        throw std::invalid_argument("fmat_mult: Matrix dimensions do not match");
    }

    float res;
    for (unsigned int i=0; i<A.rows; i++){
        for (unsigned int j=0; j<B.cols; j++){
            res = 0;
            for (unsigned int k=0; k<A.cols; k++){
                res += A.data[i][k]*B.data[k][j];
            }
            R.data[i][j] = res;
        }
    }
}

void fmat_scalar_mult(MatrixFloat &A, float a){
    for (unsigned int iRow=0; iRow<A.rows; iRow++){
        for (unsigned int iCol=0; iCol<A.cols; iCol++){
            A.data[iRow][iCol] = a * A.data[iRow][iCol];
        }
    }
}

void fmat_add(const MatrixFloat &A, const MatrixFloat &B, MatrixFloat &R){
    if (A.cols != B.cols || A.rows != B.rows || B.cols != R.cols || B.rows != R.rows){
        throw std::invalid_argument("fmat_add: Matrix dimensions do not match");
    }

    for (unsigned int i=0; i<A.rows; i++){
        for (unsigned int j=0; j<A.cols; j++){
            R.data[i][j] = A.data[i][j] + B.data[i][j];
        }
    }
}


float fmat_get_cofactor(const MatrixFloat &A, const uint16_t p, const uint16_t q){
    MatrixFloat minM(A.rows-1, A.cols-1);
    uint16_t i=0; // minor matrix row counter
    uint16_t j=0; // minor matrix col counter

    for (uint16_t iRow=0; iRow<A.rows; iRow++){
        j = 0;
        for (uint16_t iCol=0; iCol<A.cols; iCol++){
            if (iRow != p && iCol != q){
                minM.data[i][j] = A.data[iRow][iCol];
                j++;
            }
        }
        if (iRow != p){
            i++;
        }
    }
    // sign is positive if sum of row and col is even
    float sign = ((p+q) % 2 ==0) ? 1: (-1);

    return sign * fmat_det(minM); 
}

float fmat_det(const MatrixFloat &A){
    if (A.cols != A.rows){
        throw std::invalid_argument("fmat_det: A must be a square matrix");
    }
    float det = 0;
    int n = A.rows;

    if (n==1){
        // Only one element
        det = A.data[0][0];
    } else{
        for (uint16_t iCol=0; iCol < n; iCol++){
            det += A.data[0][iCol] * fmat_get_cofactor(A,0,iCol);
        }
    }

    return det;
}

void fmat_adjoint(const MatrixFloat &A, MatrixFloat &adj){
    if (A.rows != adj.rows || A.cols != adj.cols){
        throw std::invalid_argument("fmat_adjoint: A and adj must have same dimensions");
    }

    if (A.rows == 1){
        adj.data[0][0] = 1;
        return;
    }

    for (uint16_t iRow=0; iRow<A.rows; iRow++){
        for (uint16_t iCol=0; iCol<A.cols; iCol++){
            // adj is the transpose of the cofactor matrix
            // (That's why iRow and iCol are swapped)
            adj.data[iCol][iRow] = fmat_get_cofactor(A, iRow, iCol);
        }
    }
}

bool fmat_inv(const MatrixFloat &A, MatrixFloat &Ainv){
    if (A.rows != A.cols){
        throw std::invalid_argument("fmat_inv: A must be a square matrix");
    }
    if (Ainv.rows != A.rows || Ainv.cols != A.cols){
        throw std::invalid_argument("fmat_inv: Ainv and A must have the same dimensions");
    }

    float det = fmat_det(A);
    if (det==0){
        return false;
    }

    MatrixFloat adjoint(A.rows, A.cols);
    fmat_adjoint(A, adjoint);

    for (uint16_t iRow=0; iRow<A.rows; iRow++){
        for (uint16_t iCol=0; iCol<A.cols; iCol++){
            Ainv.data[iRow][iCol] = adjoint.data[iRow][iCol] / det;
        }
    }
    return true;
}

float fmat_trace(const MatrixFloat &A){
    float trace = 0;
    if (A.rows != A.cols){
        throw std::invalid_argument("fmat_trace: A must be a square matrix");
    }
    for (uint16_t i=0; i<A.rows; i++){
        trace += A.data[i][i];
    }
    return trace;
}