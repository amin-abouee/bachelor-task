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
    /// C'tor
    explicit CellLocation();

    /// C'tor
    explicit CellLocation(const int32_t x, const int32_t y);

    /// Copy C'tor
    CellLocation(const CellLocation& rhs);

    /// Move C'tor
    CellLocation(CellLocation&& rhs);

    /// Copy assignment operator
    CellLocation& operator=(const CellLocation& rhs);

    /// Move assignment operator
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

    /**
     * @brief Set the x value
     * 
     * @param x 
     */
    void setX(const int32_t x);

    /**
     * @brief Set the y value
     * 
     * @param y 
     */
    void setY(const int32_t y);

private:
    int32_t m_x;
    int32_t m_y;
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

    explicit CellData(const CellLocation loc, const double weight, const bool visited, const bool path);

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

    /// Reset all member variable to default values \n
    /// Reference: Introduction to algorithm, CLRS, chapter: 24, page: 648
    void initialize();

    /**
     * @brief Set the Loc object
     * 
     * @param x position in x (col) axis
     * @param y position in y (row) axis
     */
    void setLoc(const int32_t x, const int32_t y);

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

    /**
     * @brief Get the parent variable
     * 
     * @return const CellLocation* 
     */
    const CellLocation* getParent() const ;

    /**
     * @brief Get the weight variable
     * 
     * @return const double 
     */
    const double getWeight() const;

    /**
     * @brief Get the visited variable
     * 
     * @return true 
     * @return false 
     */
    const bool getVisited() const;

    /**
     * @brief Get the path variable
     * 
     * @return true 
     * @return false 
     */
    const bool getPath() const;


    /**
     * @brief Set the parent variable
     * 
     * @param _parent 
     */
    void setParent(const CellLocation* _parent);

    /**
     * @brief Set the weight variable
     * 
     * @param weight 
     */
    void setWeight(const double weight);

    /**
     * @brief Set the visited variable
     * 
     * @param visited 
     */
    void setVisited(const bool visited);

    /**
     * @brief Set the path variable
     * 
     * @param path 
     */
    void setPath(const bool path);



private:
    /// Pointer to parent cell
    const CellLocation* m_parent;
    /// 2D location of cell
    CellLocation m_loc;
    /// weight of current cell, used for shortest path
    double m_weight;
    /// visiting flag
    bool m_visited;
    /// possibility of path 
    bool m_path;
 };

 #endif /* __CELL_INFO_H__ */