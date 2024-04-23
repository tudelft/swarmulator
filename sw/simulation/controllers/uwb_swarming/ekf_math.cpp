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

/**
 * @brief In Matrix M, devide row by a
 */
void fmat_devide_row_by(MatrixFloat &M, const uint16_t row, const float a){
    for (uint16_t iCol=0; iCol<M.cols; iCol++){
        M.data[row][iCol] = M.data[row][iCol]/a;
    }
}

/**
 * @brief In Matrix M, subtract a*row2 from row1
 */
void fmat_subtract_row(MatrixFloat &M, const uint16_t row1, const uint16_t row2, const float a){
    for (uint16_t iCol=0; iCol<M.cols; iCol++){
        M.data[row1][iCol] = M.data[row1][iCol] - a * M.data[row2][iCol];
    }
}

/**
 * @brief In Matrix M, swap row1 with row2
 */
void fmat_swap_rows(MatrixFloat &M, const uint16_t row1, const uint16_t row2){
    float tmp;
    for (uint16_t iCol=0; iCol<M.cols; iCol++){
        tmp = M.data[row1][iCol];
        M.data[row1][iCol] = M.data[row2][iCol];
        M.data[row2][iCol] = tmp;
    }
}

// void fmat_inv(const MatrixFloat &A, MatrixFloat &Ainv){
//     if (A.cols != A.rows){
//         throw std::invalid_argument("Can't invert non-square matrix");
//     }
//     if (A.cols != Ainv.cols || A.rows != Ainv.rows){
//         throw std::invalid_argument("A and Ainv must have the same dimensions");
//     }

//     // set Ainv to identity and get temporary A for manipulations
//     MatrixFloat tmp(A.rows,A.cols);
//     for (uint16_t iRow=0; iRow<A.rows; iRow++){
//         for (uint16_t iCol=0; iCol<A.cols;iCol++){
//             tmp.data[iRow][iCol] = A.data[iRow][iCol];
//             if (iCol==iRow){
//                 Ainv.data[iRow][iCol] = 1;
//             } else{
//                 Ainv.data[iRow][iCol] = 0;
//             }
//         }
//     }

//     // Gauss-Jordan Elimination
//     // get to upper triangular
//     float v_diag; // value on diagonal
//     for (uint16_t iRow=0; iRow<tmp.rows; iRow++){
//         // diagonal to 1
//         v_diag = tmp.data[iRow][iRow];
//         if (v_diag==0){
//             uint16_t alt_row;
//             for (alt_row=iRow; alt_row<tmp.rows; alt_row++){
//                 v_diag = tmp.data[alt_row][alt_row];
//                 if (v_diag != 0){
//                     break;
//                 }
//             }

//             if (v_diag == 0){
//                 throw std::runtime_error("Matrix is not invertible");
//             } else {
//                 fmat_swap_rows(Ainv, iRow, alt_row);
//                 fmat_swap_rows(tmp, iRow, alt_row);
//             }   
//         }
//         fmat_devide_row_by(Ainv, iRow, v_diag);
//         fmat_devide_row_by(tmp, iRow, v_diag);
        
//         // subsequent rows upper triangular
//         for (uint16_t iRow2=iRow+1; iRow2<tmp.rows; iRow2++){
//             float multiple = tmp.data[iRow2][iRow]; // first value in row needs to be subtracted
//             fmat_subtract_row(Ainv, iRow2, iRow, multiple);
//             fmat_subtract_row(tmp, iRow2, iRow, multiple);
//         }
//     }
//     // back up
//     for (int16_t iRow=(tmp.rows-1); iRow>=0; iRow--){
//         for (int16_t iRow2=iRow-1; iRow2>=0; iRow2--){
//             float multiple = tmp.data[iRow2][iRow]; // last value in row needs to be subtracted
//             fmat_subtract_row(Ainv, iRow2, iRow, multiple);
//             fmat_subtract_row(tmp, iRow2, iRow, multiple);
//         }
//     }

//     MatrixFloat test(A.rows, A.cols);
//     fmat_mult(A, Ainv, test);
//     std::cout << "Matrix inverted (hopefully): (("
//             << test.data[0][0] << "," << test.data[0][1] << "," << test.data[0][2] << "),(" 
//             << test.data[1][0] << "," << test.data[1][1] << "," << test.data[1][2] << "),(" 
//             << test.data[2][0] << "," << test.data[2][1] << "," << test.data[2][2] << "))" 
//             << std::endl;
// }

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

/*
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
*/

void fmat_get_cofactor(const MatrixFloat &A, MatrixFloat &cof, const uint16_t p, const uint16_t q){
    if (cof.rows != A.rows-1 || cof.cols != A.cols-1){
        throw std::invalid_argument("Wrong matrix dimensions");
    }

    uint16_t i=0; // cofactor row counter
    uint16_t j=0; // cofactor col counter

    for (uint16_t iRow=0; iRow<A.rows; iRow++){
        j = 0;
        for (uint16_t iCol=0; iCol<A.cols; iCol++){
            if (iRow != p && iCol != q){
                cof.data[i][j] = A.data[iRow][iCol];
                j++;
            }
        }
        if (iRow != p){
            i++;
        }
    }
}

float fmat_det(const MatrixFloat &A){
    if (A.cols != A.rows){
        throw std::invalid_argument("A must be a square matrix");
    }
    float det = 0;
    int n = A.rows;

    // Only one element
    if (n==1){
        det = A.data[0][0];
    } else{
        MatrixFloat cofactor(n-1,n-1);
        int16_t sign = 1;

        for (uint16_t iCol=0; iCol < n; iCol++){
            fmat_get_cofactor(A, cofactor, 0, iCol);
            det += sign * A.data[0][iCol] * fmat_det(cofactor);
            sign = -sign;
        }
    }

    return det;
}

void fmat_adjoint(const MatrixFloat &A, MatrixFloat &adj){
    if (A.rows != adj.rows || A.cols != adj.cols){
        throw std::invalid_argument("A and adj must have same dimensions");
    }

    if (A.rows == 1){
        adj.data[0][0] = 1;
        return;
    }

    int sign = 1;
    MatrixFloat cofactor(A.rows-1, A.cols-1);

    for (uint16_t iRow=0; iRow<A.rows; iRow++){
        for (uint16_t iCol=0; iCol<A.cols; iCol++){
            fmat_get_cofactor(A, cofactor, iRow, iCol);
            // sign is positive if sum of row and col is even
            sign = ((iRow+iCol) % 2 ==0) ? 1: (-1); 
            // adj is the transpose of the cofactor matrix
            adj.data[iCol][iRow] = sign * fmat_det(cofactor);
        }
    }
}

bool fmat_inv(const MatrixFloat &A, MatrixFloat &Ainv){
    if (A.rows != A.cols){
        throw std::invalid_argument("A must be a square matrix");
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
        std::cout << "Couldn't compute trace: A must be square!\n";
        return trace;
    }
    for (uint16_t i=0; i<A.rows; i++){
        trace += A.data[i][i];
    }
    return trace;
}