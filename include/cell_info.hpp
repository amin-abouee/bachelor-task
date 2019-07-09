/**
* @file cell-information
* @brief a data container that represent the data value
*
* @date 06.07.2019
* @author Amin Abouee
*
* @section DESCRIPTION
*
*/
#ifndef __CELL_INFO_H__
#define __CELL_INFO_H__

#include <iostream>

/**
 * @brief This class represents the location of a cell inside a grid graph
 * 
 */
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

    /**
     * @brief equal binary operator overload
     * 
     * @param other the right hand side variable 
     * @return true is equal
     * @return false is not equal
     */
    bool operator==(const CellLocation& other) const;

    // void setLoc(const int32_t x, conat int32_t y);

    /**
     * @brief Get the x value
     * 
     * @return int return value of x
     */
    int X() const;

    /**
     * @brief Get the y value
     * 
     * @return int return value of y
     */
    int Y() const;


    void setX(const int32_t x);

    void setY(const int32_t y);

private:
    int32_t x;
    int32_t y;
};


// /**
// * @section DESCRIPTION
// * This class designed as a data container inside the grid graph
// * The data member variable mostly used for finding the shortest path inside a grid graph
// */

class CellData final
{
public:
    /// C'tor
    explicit CellData();

    explicit CellData(const CellLocation _loc, const double _weight, const bool _visited, const bool _path);

    /// Copy C'tor
    CellData(const CellData& rhs);

    /// Move C'tor
    CellData(CellData&& rhs);

    /// Copy assignment operator
    CellData& operator=(const CellData& rhs);

    /// Move assignment operator
    CellData& operator=(CellData&& rhs);

    /// D'tor
    ~CellData() = default;

    /// Reset all member variable to default values
    /// Reference: Introduction to algorithm, CLRS, chapter: 24, page: 648
    void initialize();

    /**
     * @brief Set the Loc object
     * 
     * @param x position in x (col) axis
     * @param y position in y (row) axis
     */
    void setLoc(const int32_t _x, const int32_t _y);

    CellLocation& getLoc();

    /**
     * @brief Get the Loc object
     * 
     * @return const CellLocation& reference to location variable
     */
    const CellLocation& getLoc() const;

    /**
     * @brief Get the Data object
     * 
     * @return CellData& 
     */
    CellData& getData();

    /**
     * @brief Get the Data object
     * 
     * @return const CellData& reference to cureent object
     */
    const CellData& getData() const;

    // CellLocation* getParent();

    const CellLocation* getParent() const ;


    const double getWeight() const;

    const bool getVisited() const;

    const bool getPath() const;


    void setParent(const CellLocation* _parent);
    // void setParent(CellLocation* _parent);


    void setWeight(double _weight);

    void setVisited(bool _visited);

    void setPath(bool _path);



private:
    const CellLocation* parent;
    CellLocation loc;
    double weight;
    bool visited;
    bool path;
 };

 #endif /* __CELL_INFO_H__ */