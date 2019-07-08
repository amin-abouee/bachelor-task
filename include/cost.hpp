/**
* @file cost.hpp
* @brief compute cost for altitude uphill and downhill movement
*
* @date 06.07.2019
* @author Amin Abouee
*
* @section DESCRIPTION
*
*   reference: http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#heuristics-for-grid-maps
*   reference: http://theory.stanford.edu/~amitp/GameProgramming/MovementCosts.html

*/
#ifndef __COST_H__
#define __COST_H__

#include <iostream>
#include <map>
#include "cell_info.hpp"

class Cost
{
public:

    enum class CostModel : unsigned int
    {
        Octile       = 0,
        Pick         = 1,
        L2           = 2,
        L1           = 3,
        LInf         = 4,
        Angle        = 5,
        DifficultyLevel = 6,
        MeanPick    = 7,
    };

    //C'tor
    explicit Cost() = default;
    //Copy C'tor
    Cost(const Cost& rhs) = default;
    //move C'tor
    Cost(Cost&& rhs) = default;
    //Copy assignment operator
    Cost& operator=(const Cost& rhs) = default;
    //move assignment operator
    Cost& operator=(Cost&& rhs) = default;
    //D'tor
    virtual ~Cost() = default;

    static std::map< std::string, CostModel > allModels;

double computeCost(const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget,
                            const CostModel& model);

double computeOCtile( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget ); 

double computePick( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget );

double computeMeanPick( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget ); 

virtual double computeL2( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget ) = 0; 

virtual double computeL1( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget ) = 0; 

virtual double computeLInf( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget ) = 0; 

virtual double computeAngle( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget ) = 0; 
                            
virtual double computeDifficultyLevel( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget ) = 0; 

 private:

 };

 #endif /* __COST_H__ */