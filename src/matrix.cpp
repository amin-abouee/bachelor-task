#include "matrix.hpp"
#include "cell_info.hpp"

template <typename T>
Matrix<T>::Matrix(const uint32_t numRows, const uint32_t numCols): m_numRows(numRows), m_numCols(numCols), m_matrix(m_numRows*numCols)
{
}

template <typename T>
Matrix<T>::Matrix(const uint32_t numRows, const uint32_t numCols, std::vector<T>& data) : m_numRows(numRows), m_numCols(numCols), m_matrix(m_numRows*numCols)
{
    if (data.size() == m_numRows*numCols)
        std::copy ( data.begin(), data.end(), m_matrix.begin() );
    else
    {
        throw std::runtime_error("The size of input vector and matrix do not match");
    }
}

template <typename T>
const T & Matrix<T>::operator()(const uint32_t row, const uint32_t column)  const
{
    return m_matrix[row * m_numCols + column];
}

template <typename T>
T & Matrix<T>::operator()(const uint32_t row, const uint32_t column) 
{
    return m_matrix[row * m_numCols + column];
}

template <typename T>
const std::vector<T> & Matrix<T>::getMatrix() const
{
    return m_matrix;
}

template <typename T>
std::vector<T> & Matrix<T>::getMatrix()
{
    return m_matrix;
}

template <typename T>
uint32_t Matrix<T>::getRows() const
{
    return m_numRows;
}

template <typename T>
uint32_t Matrix<T>::getCols() const
{
    return m_numCols;
}

template <typename T>
T* Matrix<T>::data()
{
    return m_matrix.data();
}

template <typename T>
const T* Matrix<T>::data() const
{
    return m_matrix.data();
}

template <typename T>
uint32_t Matrix<T>::getTotalSize() const
{
    return m_numCols * m_numRows;
}

template class Matrix<uint8_t>;
template class Matrix<uint16_t>;
template class Matrix<uint32_t>;
template class Matrix<CellData>;