/** 
 * @file matrix.hpp
 * @author  Amin Abouee
 * @date 04.06.2019
 *
 * @section DESCRIPTION
 */


#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <iostream>
#include <vector>
#include <limits>

/**
 * @brief Matrix representation for 1D data with size n*n \n
 * This wrapper class represents 1D vector with size n*n as a 2D matrix with size n by n
 * 
 * @tparam T Template type name (unit8 or CellData)
 */
template <typename  T>
class Matrix final
{
public:
    /// C'tor
    explicit Matrix(const uint32_t numRows, const uint32_t numCols);

    /// C'tor
    explicit Matrix(const uint32_t numRows, const uint32_t numCols, std::vector<T>& data);

    /// D'tor
    virtual ~Matrix() = default;

    /// Copy C'tor
    Matrix(const Matrix & rhs) = default;

    /// Move C'tor
    Matrix(Matrix && rhs) = default;
    
    /// Copy assignment operator
    Matrix &operator=(const Matrix & rhs) = default;

    /// Move assignment operator
    Matrix &operator=(Matrix && rhs) = default;

    /**
     * @brief overload operator() 
     * 
     * @param row y axis
     * @param column x axis
     * @return const T&
     */
    const T & operator()(const uint32_t row, const uint32_t column) const;

    /**
     * @brief overload operator ()
     * 
     * @param row y axis
     * @param column x axis
     * @return T& return
     */
    T & operator()(const uint32_t row, const uint32_t column);

    /**
     * @brief Get the Matrix object
     * 
     * @return const std::vector<T>& 
     */
    const std::vector<T> & getMatrix() const;

    /**
     * @brief Get the Matrix object
     * 
     * @return std::vector<T>& 
     */
    std::vector<T> & getMatrix();

    /**
     * @brief Get the number of rows
     * 
     * @return uint32_t 
     */
    uint32_t getRows() const;

    /**
     * @brief Get the number of columns
     * 
     * @return uint32_t 
     */
    uint32_t getCols() const;

    /**
     * @brief A raw pointer to m_matrix variable
     * 
     * @return T* 
     */
    T* data();

    /**
     * @brief A const raw pointer to m_matrix variable
     * 
     * @return const T* 
     */
    const T* data() const;

    /**
     * @brief Get the number of cells (rows * cols)
     * 
     * @return uint32_t 
     */
    uint32_t getTotalSize() const;

    //TODO: set function


    // friend std::ostream& operator>>( std::ostream& os, Matrix& mat);

private:
    /// number of rows
    std::uint32_t m_numRows;
    /// number of cols
    std::uint32_t m_numCols;
    /// data vecto
    std::vector<T> m_matrix;
};

#endif /* __MATRIX_H__ */
