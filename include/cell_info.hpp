/**
* @file cell-information
* @brief a data container that represent the data value
*
* @date 06.07.2019
* @author Amin Abouee
*
* @section DESCRIPTION
*
*
*/
#ifndef __CELL_INFO_H__
#define __CELL_INFO_H__

#include <iostream>

class CellLocation final
{
public:
    //C'tor
    explicit CellLocation();

    //C'tor
    explicit CellLocation(const int32_t x, const int32_t y);
    //Copy C'tor
    CellLocation(const CellLocation& rhs);

    //move C'tor
    CellLocation(CellLocation&& rhs);

    //Copy assignment operator
    CellLocation& operator=(const CellLocation& rhs);

    //move assignment operator
    CellLocation& operator=(CellLocation&& rhs);

    //D'tor
    ~CellLocation() = default;

    bool operator==(const CellLocation& other) const;

public:
    int32_t x;
    int32_t y;
};


// /**
// * @section DESCRIPTION
// *
// *
// */

class CellData final
{
public:
    //C'tor
    explicit CellData();

    explicit CellData(const CellLocation _loc, const double _weight, const bool _visited, const bool _path);

    //Copy C'tor
    CellData(const CellData& rhs);

    //move C'tor
    CellData(CellData&& rhs);

    //Copy assignment operator
    CellData& operator=(const CellData& rhs);

    //move assignment operator
    CellData& operator=(CellData&& rhs);

    //D'tor
    ~CellData() = default;

    void initialize();

    void setLoc(int32_t x, int32_t y);

    CellLocation& getLoc();

    const CellLocation& getLoc() const;

    CellData& getData();

    const CellData& getData() const;


public:
    CellLocation* parent;
    CellLocation loc;
    double weight;
    bool visited;
    bool path;
 };

 #endif /* __CELL_INFO_H__ */