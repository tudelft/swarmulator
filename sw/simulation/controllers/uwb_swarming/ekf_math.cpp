#include "ekf_math.h"

#include <iostream>


void fmat_trans(const MatrixFloat &A, MatrixFloat &AT){
    if (A.cols != AT.rows || A.rows != AT.cols){
        throw std::invalid_argument("Matrix dimensions do not match");
    }

    for (unsigned int i=0; i<A.rows; i++){
        for (unsigned int j=0; j<A.cols; j++){
            AT.data[j][i] = A.data[i][j];
        }
    }
}

void fmat_mult(const MatrixFloat &A, const MatrixFloat &B, MatrixFloat &R){
    if (A.cols != B.rows || A.rows != R.rows || B.cols != R.cols){
        throw std::invalid_argument("Matrix dimensions do not match");
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

void fmat_add(const MatrixFloat &A, const MatrixFloat &B, MatrixFloat &R){
    if (A.cols != B.cols || A.rows != B.rows || B.cols != R.cols || B.rows != R.rows){
        throw std::invalid_argument("Matrix dimensions do not match");
    }

    for (unsigned int i=0; i<A.rows; i++){
        for (unsigned int j=0; j<A.cols; j++){
            R.data[i][j] = A.data[i][j] + B.data[i][j];
        }
    }
}

bool LUdecomposition(const MatrixFloat &A, MatrixFloat &L, MatrixFloat &U) {
    if (A.rows != A.cols || L.rows != L.cols || U.rows != U.cols){
        std::cout << "LU: A, L and U must be square matrices!" << std::endl;
        return false;
    }
    if (L.rows != A.rows || U.rows != A.rows){
        std::cout << "LU: A, L and U must have the same dimensions!" << std::endl;
        return false;
    }

    int n = A.rows;
   
    int i = 0, j = 0, k = 0;
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            if (j < i)
                L.data[j][i] = 0;
            else {
                L.data[j][i] = A.data[j][i];
                for (k = 0; k < i; k++) {
                    L.data[j][i] = L.data[j][i] - L.data[j][k] * U.data[k][i];
                }
            }
        }
        for (j = 0; j < n; j++) {
            if (j < i)
                U.data[i][j] = 0;
            else if (j == i)
                U.data[i][j] = 1;
            else {
                U.data[i][j] = A.data[i][j] / L.data[i][i];
                for (k = 0; k < i; k++) {
                    U.data[i][j] = U.data[i][j] - ((L.data[i][k] * U.data[k][j]) / L.data[i][i]);
                }
            }
        }
    }
    return true;
}

float fmat_det(const MatrixFloat &A){
    float det = 0;
    int n = A.rows;
    MatrixFloat L(n,n);
    MatrixFloat U(n,n);
    if(LUdecomposition(A, L, U)){
        det = 1;
        for (int i=0; i<n; i++){
            det = det*L.data[i][i]*U.data[i][i];
        }
    }
    return det;
}