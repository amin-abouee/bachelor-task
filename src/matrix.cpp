#include "matrix.hpp"

template <typename T>
Matrix<T>::Matrix(const uint32_t numRows, const uint32_t numCols): m_numRows(numRows), m_numCols(numCols) noexcept
{

}

template <typename T>
const uint8_t & Matrix<T>::operator()(const uint32_t row, const uint32_t column)  const noexcept
{
    return m_matrix[row * m_numCols + column];
}

template <typename T>
uint8_t & Matrix<T>::operator()(const uint32_t row, const uint32_t column)  noexcept
{
    return m_matrix[row * m_numCols + column];
}

// template <typename T>
// uint8_t Matrix<T>::operator()(const uint32_t row, const uint32_t column)  const noexcept
// {
//     return m_matrix[row * m_numCols + column];
// }

// template <typename T>
// const uint8_t & Matrix<T>::operator[](const uint32_t row, const uint32_t column)  const noexcept
// {
//     return m_matrix[row * m_numCols + column];
// }

// template <typename T>
// uint8_t & Matrix<T>::operator[](const uint32_t row, const uint32_t column)  noexcept
// {
//     return m_matrix[row * m_numCols + column];
// }

// template <typename T>
// uint8_t Matrix<T>::operator[](const uint32_t row, const uint32_t column)  const noexcept
// {
//     return m_matrix[row * m_numCols + column];
// }

template <typename T>
const uint32_t Matrix<T>::getSizeRows() const noexcept
{
    return m_numRows;
}

template <typename T>
const uint32_t Matrix<T>::getSizeCols() const noexcept
{
    return m_numCols;
}

template <typename T>
std::ostream& operator>>( std::ostream& out, Matrix<T>& mat)
{
    const uint32_t sizeCols = mat.getSizeCols();
    const uint32_t sizeRows = mat.getSizeRows();


    for (int c = 0; c < sizeCols; c++){
        for (int r = 0; r < sizeRows; r++){
            out << mat[c][r] << " ";
        }
        out << std::endl;
    }
    return out;
}