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
    x = rhs.X();
    y = rhs.Y();
}
//move C'tor
CellLocation::CellLocation(CellLocation&& rhs)
{
    x = (std::move(rhs.X()));
    y = (std::move(rhs.Y()));
}

//Copy assignment operator
CellLocation& CellLocation::operator=(const CellLocation& rhs)
{
    x = rhs.X();
    y = rhs.Y();
    return *this;
}

//move assignment operator
CellLocation& CellLocation::operator=(CellLocation&& rhs)
{
    x = (std::move(rhs.X()));
    y = (std::move(rhs.Y()));
    return *this;
}

bool CellLocation::operator==(const CellLocation& other) const
{
    return (other.X() == x && other.Y() == y);
}

int CellLocation::X() const
{
    return x;
}

int CellLocation::Y() const
{
    return y;
}

void CellLocation::setX(const int32_t _x)
{
    x = _x;
}

void CellLocation::setY(const int32_t _y)
{
    y = _y;
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
    // parent = rhs.getParent();
    loc = rhs.getLoc();
    weight = rhs.getWeight();
    visited = rhs.getVisited();
    path = rhs.getPath();
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
    parent = rhs.getParent();
    loc = rhs.getLoc();
    weight = rhs.getWeight();
    visited = rhs.getVisited();
    path = rhs.getPath();
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

void CellData::setLoc(const int32_t _x, const int32_t _y)
{
    this->loc.setX(_x);
    this->loc.setY(_y);
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

// const CellData& CellData::getData() const
// {
//     return *this;
// }

const CellLocation* CellData::getParent() const
{
    return parent;
}

// CellLocation* CellData::getParent()
// {
//     return parent;
// }


const double CellData::getWeight() const
{
    return weight;
}

const bool CellData::getVisited() const
{
    return visited;
}

const bool CellData::getPath() const
{
    return path;
}

// void CellData::setLoc(const int32_t x, const int32_t y)
// {
//     this->loc.setX() = x;
//     this->loc.y = y;
// }

void CellData::setParent(const CellLocation* _parent)
{
    parent = _parent;
}

void CellData::setWeight(double _weight)
{
    weight = _weight;
}

void CellData::setVisited(bool _visited)
{
    visited = _visited;
}

void CellData::setPath(bool _path)
{
    path = _path;
}