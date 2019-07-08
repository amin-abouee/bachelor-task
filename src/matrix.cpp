#include "matrix.hpp"
#include "cell_info.hpp"

template <typename T>
Matrix<T>::Matrix(const uint32_t numRows, const uint32_t numCols): m_numRows(numRows), m_numCols(numCols), m_matrix(m_numRows*numCols)
{

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
uint32_t Matrix<T>::getTotalSize() const
{
    return m_numCols * m_numRows;
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


template class Matrix<uint8_t>;
template class Matrix<CellData>;