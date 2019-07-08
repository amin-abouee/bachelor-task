/**
* @file up_hill_cost,.hpp
* @brief computed cost for down
*
* @date 
* @author Amin Abouee
*
* @section DESCRIPTION
*
*
*/
#ifndef __DOWN_HILL_COST_H__
#define __DOWN_HILL_COST_H__

#include <iostream>
#include "cost.hpp"


class DownHillCost final : public Cost
{
public:
    //C'tor
    explicit DownHillCost();
    //Copy C'tor
    DownHillCost(const DownHillCost& rhs) = default;
    //move C'tor
    DownHillCost(DownHillCost&& rhs) = default;
    //Copy assignment operator
    DownHillCost& operator=(const DownHillCost& rhs) = default;
    //move assignment operator
    DownHillCost& operator=(DownHillCost&& rhs) = default;
    //D'tor
    ~DownHillCost() = default;

    double computeL2( const CellLocation& source, 
                                const uint8_t elevationSource, 
                                const CellLocation& target, 
                                const uint8_t elevationTarget ); 

    double computeL1( const CellLocation& source, 
                                const uint8_t elevationSource, 
                                const CellLocation& target, 
                                const uint8_t elevationTarget ); 

    double computeLInf( const CellLocation& source, 
                                const uint8_t elevationSource, 
                                const CellLocation& target, 
                                const uint8_t elevationTarget ); 

    double computeAngle( const CellLocation& source, 
                                const uint8_t elevationSource, 
                                const CellLocation& target, 
                                const uint8_t elevationTarget ); 
                                
    double computeDifficultyLevel( const CellLocation& source, 
                                const uint8_t elevationSource, 
                                const CellLocation& target, 
                                const uint8_t elevationTarget ); 

 private:

 };

 #endif /* __DOWN_HILL_COST_H__ */