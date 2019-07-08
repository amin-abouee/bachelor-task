/**
* @file up_hill_cost,.hpp
* @brief computed cost for uphill
*
* @date 
* @author Amin Abouee
*
* @section DESCRIPTION
*
*
*/
#ifndef __UP_HILL_COST_H__
#define __UP_HILL_COST_H__

#include <iostream>
#include "cost.hpp"
// #include "cell_info.hpp"

class UpHillCost final : public Cost
{
public:
    //C'tor
    explicit UpHillCost();
    //Copy C'tor
    UpHillCost(const UpHillCost& rhs) = default;
    //move C'tor
    UpHillCost(UpHillCost&& rhs) = default;
    //Copy assignment operator
    UpHillCost& operator=(const UpHillCost& rhs) = default;
    //move assignment operator
    UpHillCost& operator=(UpHillCost&& rhs) = default;
    //D'tor
    ~UpHillCost() = default;

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

 #endif /* __UP_HILL_COST_H__ */