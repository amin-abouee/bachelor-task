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
#include <limits>

struct GridLocation 
{
    int32_t x, y;
    GridLocation( int32_t x, int32_t y) :  x( x) , y( y) { };
};

struct GridWithWeights
{
    GridLocation* parent;
    GridLocation loc;
    double weight;
    bool visited;
    bool path;
    GridWithWeights () : loc(0,0), parent(nullptr), weight(100000.0), visited(false), path{false} {};
    GridWithWeights (GridLocation _loc, GridLocation* _parent, double _weight) : loc(_loc), parent(_parent), weight(_weight) {};
    void setLoc(int32_t x, int32_t y)
    {
        loc.x = x;
        loc.y = y;
    };

    void initialize()
    {
        parent = nullptr;
        weight = std::numeric_limits<double>::max();
        visited = false;
        path = false;
    }
};

template <typename  T>
class Matrix final
{
public:
    //C'tor
    explicit Matrix(const uint32_t numRows, const uint32_t numCols);
    //D'tor
    virtual ~Matrix() = default;

    //Copy C'tor
    Matrix(const Matrix & rhs) = default;
    //move C'tor
    Matrix(Matrix && rhs) = default;
    //Copy assignment operator
    Matrix &operator=(const Matrix & rhs) = default;
    //move assignment operator
    Matrix &operator=(Matrix && rhs) = default;

    const T & operator()(const uint32_t row, const uint32_t column) const;

    T & operator()(const uint32_t row, const uint32_t column);


    // uint8_t operator()(const uint32_t row, const uint32_t column) const;

    // const uint8_t & operator[](const uint32_t row, const uint32_t column) const;

    // uint8_t & operator[](const uint32_t row, const uint32_t column);

    // uint8_t operator[](const uint32_t row, const uint32_t column) const;

    const std::vector<T> & getMatrix() const;

    std::vector<T> & getMatrix();

    uint32_t getRows() const;

    uint32_t getCols() const;

    T* data();

    uint32_t getTotalSize() const;


    friend std::ostream& operator>>( std::istream& os, Matrix& mat);

private:
    std::uint32_t m_numRows, m_numCols;
    std::vector<T> m_matrix;
};

// #include "matrix.cpp"
#endif /* __MATRIX_H__ */
