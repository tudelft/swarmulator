#ifndef EKF_MATH_H
#define EKF_MATH_H

#include <vector>
#include <stdexcept>

struct MatrixFloat{
    MatrixFloat(unsigned int rows, unsigned int cols){
        this->rows = rows;
        this->cols = cols;
        for (unsigned int i=0; i<this->rows; i++){
            this->data.push_back(std::vector<float>(this->cols));
        }
    }

    unsigned int rows;
    unsigned int cols;
    std::vector<std::vector<float>> data;
};

void fmat_trans(const MatrixFloat &A, MatrixFloat &AT);

void fmat_mult(const MatrixFloat &A, const MatrixFloat &B, MatrixFloat &R);

void fmat_add(const MatrixFloat &A, const MatrixFloat &B, MatrixFloat &R);

bool fmat_inv(const MatrixFloat &A, MatrixFloat &Ainv);

float fmat_det(const MatrixFloat &A);

float fmat_trace(const MatrixFloat &A);



#endif // EKF_MATH_H