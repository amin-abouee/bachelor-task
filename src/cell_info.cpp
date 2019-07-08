#include "cell_info.hpp"
#include <limits>

CellLocation::CellLocation(): x(0), y(0) 
{

}

CellLocation::CellLocation(const int32_t _x, const int32_t _y): x(_x), y(_y) 
{

}

//Copy C'tor
CellLocation::CellLocation(const CellLocation& rhs)
{
    x = rhs.x;
    y = rhs.y;
}
//move C'tor
CellLocation::CellLocation(CellLocation&& rhs)
{
    x = (std::move(rhs.x));
    y = (std::move(rhs.y));
}

//Copy assignment operator
CellLocation& CellLocation::operator=(const CellLocation& rhs)
{
    x = rhs.x;
    y = rhs.y;
    return *this;
}

//move assignment operator
CellLocation& CellLocation::operator=(CellLocation&& rhs)
{
    x = (std::move(rhs.x));
    y = (std::move(rhs.y));
    return *this;
}

bool CellLocation::operator==(const CellLocation& other) const
{
    return (other.x == x && other.y == y);
}



CellData::CellData() : parent(nullptr), loc(0, 0), weight(std::numeric_limits<double>::max()), visited(false), path(false)
{

}

CellData::CellData(const CellLocation _loc, const double _weight, const bool _visited, const bool _path) : parent(nullptr), loc(_loc), weight(_weight), visited(_visited), path(_path)
{

}

//Copy C'tor
CellData::CellData(const CellData& rhs)
{
    parent = rhs.parent;
    loc = rhs.loc;
    weight = rhs.weight;
    visited = rhs.visited;
    path = rhs.path;
}

//move C'tor
CellData::CellData(CellData&& rhs)
{
    parent = (std::move(rhs.parent));
    loc = (std::move(rhs.loc));
    weight = (std::move(rhs.weight));
    visited = (std::move(rhs.visited));
    path = (std::move(rhs.path));
}

//Copy assignment operator
CellData& CellData::operator=(const CellData& rhs)
{
    parent = rhs.parent;
    loc = rhs.loc;
    weight = rhs.weight;
    visited = rhs.visited;
    path = rhs.path;
    return *this;
}

//move assignment operator
CellData& CellData::operator=(CellData&& rhs)
{
    parent = (std::move(rhs.parent));
    loc = (std::move(rhs.loc));
    weight = (std::move(rhs.weight));
    visited = (std::move(rhs.visited));
    path = (std::move(rhs.path));
    return *this;
}


void CellData::initialize()
{
    parent = nullptr;
    weight = std::numeric_limits<double>::max();
    visited = false;
    path = false;
}

void CellData::setLoc(const int32_t x, const int32_t y)
{
    loc.x = x;
    loc.y = y;
}

CellLocation& CellData::getLoc()
{
    return loc;
}

const CellLocation& CellData::getLoc() const
{
    return loc;
}

CellData& CellData::getData()
{
    return *this;
}

const CellData& CellData::getData() const
{
    return *this;
}