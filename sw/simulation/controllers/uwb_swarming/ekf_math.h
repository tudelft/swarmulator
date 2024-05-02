/**
 * @file ekf_math.h
 * @author Sven Pfeiffer, MAVLab, TU Delft
 * @date 23 Apr 2024
 * @brief Matrix structure and operations needed in the ekf
 * 
 * This header contains function declarations for common matrix 
 * operations that are required by the relative localization ekf
 * or the AgentInitializer
 */

#ifndef EKF_MATH_H
#define EKF_MATH_H

#include <vector>
#include <stdexcept>

/**
 * @brief structure to represent a matrix with float values
 */
struct MatrixFloat{
    /**
     * @brief initializes a new matrix with specifie number
     * of rows and columns
     * @param[in] rows: number of rows
     * @param[in] cols: number of columns
     */
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


/**
 * @brief set all elements of a matrix to zero
 * @param A: Matrix that will be set to zero
 */
void fmat_set_zero(MatrixFloat &A);

/**
 * @brief Set all elements in A, so that A is an identity matrix
 * @param A: Matrix that will be set to the identity matrix
 * @throws std::invalid_argument if A is not square
 */
void fmat_set_identity(MatrixFloat &A);

/**
 * @brief Calculate and return the transpose of a matrix
 * @param[in] A: Original Matrix
 * @param[out] AT: Transpose of A
 */
void fmat_trans(const MatrixFloat &A, MatrixFloat &AT);

/**
 * @brief Multiply two matrices
 * @param[in] A: First Matrix
 * @param[in] B: Second Matrix
 * @param[out] R: Result of the matrix multiplication A*B
 */
void fmat_mult(const MatrixFloat &A, const MatrixFloat &B, MatrixFloat &R);

/**
 * @brief Multiply each element of a matrix with a scalar
 * @param A: The matrix
 * @param a: The scalar
 */
void fmat_scalar_mult(MatrixFloat &A, float a);
/**
 * @brief Add two matrices
 * @param[in] A: First Matrix
 * @param[in] B: Second Matrix
 * @param[out] R: Result of the addition A+B
 */
void fmat_add(const MatrixFloat &A, const MatrixFloat &B, MatrixFloat &R);

/**
 * @brief Invert a matrix using its adjoint and determinant
 * @param[in] A: Original matrix
 * @param[out] Ainv: Inverse of A
 * @return True if the matrix could be inverted
 * @throws std::invalid_argument if A is not square
 * @throws std::invalid_argument if Ainv has different dimensions from A
 */
bool fmat_inv(const MatrixFloat &A, MatrixFloat &Ainv);

/**
 * @brief Return the cofactor corresponding to the element A_pq
 * 
 * The cofactor of an element in a matrix is the determinant of 
 * the matrix obtained by removing the row and column containing the element,
 * multiplied by +1 or -1 depending on the position in the matrix
 * @param[in] A: Original matrix
 * @param[in] p: Row of the element
 * @param[in] q: Column of the element
 * @return Cofactor of the element
 */
float fmat_get_cofactor(const MatrixFloat &A, const uint16_t p, const uint16_t q);

/**
 * @brief Calculate the adjoint of a matrix
 * 
 * The adjoint of a matrix is the transpose of its cofactor matrix
 * @param[in] A: Original Matrix
 * @param[out] adj: Adjoint of A
 * @throws std::invalid_argument if A and adj don't have the same dimensions
 */
void fmat_adjoint(const MatrixFloat &A, MatrixFloat &adj);

/**
 * @brief Calculate the determinant of a matrix recursively
 * @param[in] A: Original Matrix
 * @return Determinant of A
 * @throws std::invalid_argument if A is not square
 */
float fmat_det(const MatrixFloat &A);

/**
 * @brief Calculate the trace of a matrix
 * @param[in] A: Original Matrix
 * @return Trace of the matrix
 * @throws std::invalid_argument if A is not square
 */
float fmat_trace(const MatrixFloat &A);



#endif // EKF_MATH_H