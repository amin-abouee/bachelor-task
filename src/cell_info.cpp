#include "cell_info.hpp"
#include <limits>

CellLocation::CellLocation(): m_x(0), m_y(0) 
{

}

CellLocation::CellLocation(const int32_t x, const int32_t y): m_x(x), m_y(y) 
{

}

CellLocation::CellLocation(const CellLocation& rhs)
{
    m_x = rhs.X();
    m_y = rhs.Y();
}

CellLocation::CellLocation(CellLocation&& rhs)
{
    m_x = (std::move(rhs.X()));
    m_y = (std::move(rhs.Y()));
}

CellLocation& CellLocation::operator=(const CellLocation& rhs)
{
    m_x = rhs.X();
    m_y = rhs.Y();
    return *this;
}

CellLocation& CellLocation::operator=(CellLocation&& rhs)
{
    m_x = (std::move(rhs.X()));
    m_y = (std::move(rhs.Y()));
    return *this;
}

bool CellLocation::operator==(const CellLocation& other) const
{
    return (other.X() == m_x && other.Y() == m_y);
}

bool CellLocation::operator!=(const CellLocation& other) const
{
    return (other.X() != m_x || other.Y() != m_y);
}

int CellLocation::X() const
{
    return m_x;
}

int CellLocation::Y() const
{
    return m_y;
}

void CellLocation::setX(const int32_t x)
{
    m_x = x;
}

void CellLocation::setY(const int32_t y)
{
    m_y = y;
}

std::ostream& operator<<( std::ostream& os, CellLocation& loc)
{
    os << "[" << loc.X() << " , " << loc.Y()<< "] ";
    return os; 
}

CellData::CellData() : m_parent(nullptr), m_loc(0, 0), m_weight(std::numeric_limits<double>::max()), m_visited(false), m_path(false)
{

}

CellData::CellData(const CellLocation loc, const double weight, const bool visited, const bool path) : m_parent(nullptr), m_loc(loc), m_weight(weight), m_visited(visited), m_path(path)
{

}

CellData::CellData(const CellData& rhs)
{
    // parent = rhs.getParent();
    m_loc = rhs.getLoc();
    m_weight = rhs.getWeight();
    m_visited = rhs.getVisited();
    m_path = rhs.getPath();
}

CellData::CellData(CellData&& rhs)
{
    m_parent = (std::move(rhs.getParent()));
    m_loc = (std::move(rhs.getLoc()));
    m_weight = (std::move(rhs.getWeight()));
    m_visited = (std::move(rhs.getVisited()));
    m_path = (std::move(rhs.getPath()));
}

CellData& CellData::operator=(const CellData& rhs)
{
    m_parent = rhs.getParent();
    m_loc = rhs.getLoc();
    m_weight = rhs.getWeight();
    m_visited = rhs.getVisited();
    m_path = rhs.getPath();
    return *this;
}

CellData& CellData::operator=(CellData&& rhs)
{
    m_parent = (std::move(rhs.getParent()));
    m_loc = (std::move(rhs.getLoc()));
    m_weight = (std::move(rhs.getWeight()));
    m_visited = (std::move(rhs.getVisited()));
    m_path = (std::move(rhs.getPath()));
    return *this;
}


void CellData::initialize()
{
    m_parent = nullptr;
    m_weight = std::numeric_limits<double>::max();
    m_visited = false;
}

void CellData::setLoc(const int32_t _x, const int32_t _y)
{
    m_loc.setX(_x);
    m_loc.setY(_y);
}

CellLocation& CellData::getLoc()
{
    return m_loc;
}

const CellLocation& CellData::getLoc() const
{
    return m_loc;
}

CellData& CellData::getData()
{
    return *this;
}

const CellLocation* CellData::getParent() const
{
    return m_parent;
}

const double CellData::getWeight() const
{
    return m_weight;
}

const bool CellData::getVisited() const
{
    return m_visited;
}

const bool CellData::getPath() const
{
    return m_path;
}

void CellData::setParent(const CellLocation* parent)
{
    m_parent = parent;
}

void CellData::setWeight(const double weight)
{
    m_weight = weight;
}

void CellData::setVisited(const bool visited)
{
    m_visited = visited;
}

void CellData::setPath(const bool path)
{
    m_path = path;
}

std::ostream& operator<<( std::ostream& os, CellData& data)
{
    os << std::boolalpha << "Loc: " << data.getLoc() << ", Weight: " << data.getWeight() << ", Vis: " << data.getVisited() << ", Path: " << data.getPath();
    return os; 
}