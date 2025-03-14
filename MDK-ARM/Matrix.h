//
// Created by Administrator on 25-1-4.
//

/*
 * 经过测试直接调用arm_math库和数组，用时为A
 * 通过Matrix类封装arm_math库和数组，用时为B=1.18A左右
 * 通过Matrix类封装直接用数组进行矩阵运算，用时为C=1.2B~1.3B左右
 */

#ifndef DM_42_MATRIX_H
#define DM_42_MATRIX_H

#include <concepts>

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"

#ifdef __cplusplus
}
#endif
//template<size_t ROWS = 0, size_t COLS = 0>
//concept StaticStorage = (ROWS != 0 && COLS != 0);
//template<size_t ROWS = 0, size_t COLS = 0>
//concept DynamicStorage = (ROWS == 0 || COLS == 0);
template<uint32_t ROWS = 0, uint32_t COLS = 0>
class Matrix {
public:
	  arm_matrix_instance_f32 matrix {};
    float data[ROWS][COLS] {};
			
    Matrix() {
        arm_mat_init_f32(&matrix, ROWS, COLS, reinterpret_cast<float32_t *>(data));
    }

    Matrix(float data[ROWS][COLS])
        : Matrix() {
        memcmp(this->data, data, ROWS * COLS * sizeof(float));
    }

    Matrix(const Matrix& other)
        : Matrix() {
        memcpy(data, other.data, ROWS * COLS * sizeof(float));
    }

    // 重载赋值运算符处理nullptr
    void operator=(std::nullptr_t) {
        matrix.pData = nullptr;
    }

    bool operator==(std::nullptr_t) {
        return matrix.pData == nullptr;
    }

    constexpr uint32_t get_row() { return ROWS; }

    constexpr uint32_t get_col() { return COLS; }

    float& operator()(uint32_t row, uint32_t col) {
//        static_assert(row >= ROWS);
//        static_assert(col >= COLS);
        return data[row][col];
    }

    const float& operator()(uint32_t row, uint32_t col) const {
//        static_assert(row<ROWS);
//        static_assert(col<COLS);
        return data[row][col];
    }

    //    template<uint32_t row = ROWS, uint32_t col = COLS>
    Matrix operator+(const Matrix& other) const {
        Matrix result;
        arm_mat_add_f32(&matrix, &other.matrix, &result.matrix);
        return result;
    }

    //    template<uint32_t row = ROWS, uint32_t col = COLS>
    Matrix operator-(const Matrix& other) const {
        Matrix result;
        arm_mat_sub_f32(&matrix, &other.matrix, &result.matrix);
        return result;
    }

    template<uint32_t row, uint32_t col>
    Matrix<ROWS, col> operator*(const Matrix<row, col>& other) const requires(COLS == row) {
        Matrix<ROWS, col> result;
        arm_mat_mult_f32(&matrix, &other.matrix, &result.matrix);
        return result;
    }

    //    template<uint32_t row = ROWS, uint32_t col = COLS>
    Matrix operator*(float scale) const {
        Matrix result;
        arm_mat_scale_f32(&matrix, scale, &result.matrix);
        return result;
    }

    friend Matrix<ROWS, COLS> operator*(float scale,const Matrix<ROWS, COLS>& other) {
        Matrix result;
        arm_mat_scale_f32(&other.matrix, scale, &result.matrix);
        return result;
    }

    //    template<uint32_t row = ROWS, uint32_t col = COLS>
    Matrix operator/(float scale) const {
        Matrix result;
        arm_mat_scale_f32(&matrix, 1.0f / scale, &result.matrix);
        return result;
    }

    //    template<uint32_t row = ROWS, uint32_t col = COLS>
    Matrix<COLS, ROWS> transpose() const {
        Matrix<COLS, ROWS> result;
        arm_mat_trans_f32(&matrix, &result.matrix);
        return result;
    }

    //    template<uint32_t row = ROWS, uint32_t col = COLS>
    Matrix<COLS, ROWS> operator~() {
        Matrix<COLS, ROWS> result;
        arm_mat_trans_f32(&matrix, &result.matrix);
        return result;
    }

    void operator+=(const Matrix& other) {
        arm_mat_add_f32(&matrix, &other.matrix, &matrix);
    }

    void operator-=(const Matrix& other) {
        arm_mat_sub_f32(&matrix, &other.matrix, &matrix);
    }

    void operator*=(const Matrix& other) requires(ROWS == COLS) {
        arm_mat_mult_f32(&matrix, &other.matrix, &matrix);
    }

    void operator*=(float scale) {
        arm_mat_scale_f32(&matrix, scale, &matrix);
    }

    void operator/=(float scale) {
        arm_mat_scale_f32(&matrix, 1.0f / scale, &matrix);
    }

    arm_status inv(Matrix& result)
        requires(ROWS == COLS)
    {
        arm_matrix_instance_f32 matrix_bak = {ROWS,COLS};
			
        float data_bak[ROWS][COLS];
        memcpy(data_bak, data, ROWS * COLS * sizeof(float));
				for(int i = 0;i < ROWS;++i) {
					for(int j = 0;j< COLS;++j) {
						result.data[i][j] = 0;
					}
				}
        matrix_bak.pData = reinterpret_cast<float *>(data_bak);
        arm_status ret = arm_mat_inverse_f32(&matrix_bak, &result.matrix);
				for(;;)
					break;
        return ret;
    }


private:
};

#endif //DM_42_MATRIX_H
