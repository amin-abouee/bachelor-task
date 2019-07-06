/** 
 * @file matrix.hpp
 * @author  Amin Abouee
 * @date 04.06.2019
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * https://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 *
 */

#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <iostream>
#include <vector>

template <typename  T>
class Matrix final
{
public:
    //C'tor
    explicit Matrix(const uint32_t numRows, const uint32_t numCols) noexcept;
    //D'tor
    virtual ~Matrix() noexcept = default;

    //Copy C'tor
    Matrix(const Matrix & rhs) noexcept = default;
    //move C'tor
    Matrix(Matrix && rhs) noexcept = default;
    //Copy assignment operator
    Matrix &operator=(const Matrix & rhs) noexcept = default;
    //move assignment operator
    Matrix &operator=(Matrix && rhs) noexcept = default;

    const uint8_t & operator()(const uint32_t row, const uint32_t column) const noexcept;

    uint8_t & operator()(const uint32_t row, const uint32_t column) noexcept;

    // uint8_t operator()(const uint32_t row, const uint32_t column) const noexcept;

    // const uint8_t & operator[](const uint32_t row, const uint32_t column) const noexcept;

    // uint8_t & operator[](const uint32_t row, const uint32_t column) noexcept;

    // uint8_t operator[](const uint32_t row, const uint32_t column) const noexcept;

    // size_t getRows() const
    // {
    //     return rows;
    // }
    // size_t getColumns() const
    // {
    //     return columns;
    // }

    const uint32_t getSizeRows() const noexcept;

    const uint32_t getSizeCols() const noexcept;



    friend std::ostream& operator>>( std::istream& os, Matrix& mat);

private:
    std::uint32_t m_numRows, m_numCols;
    std::vector<uint8_t> m_matrix;
};

#include "matrix.hpp"
#endif /* __MATRIX_H__ */
