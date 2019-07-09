#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <cmath>
#include "cell_info.hpp"
#include "matrix.hpp"
#include "cost.hpp"
#include "up_hill_cost.hpp"
#include "down_hill_cost.hpp"

struct CellLocationTest : testing::Test
{
  std::shared_ptr <CellLocation> cell = std::make_shared <CellLocation> (10, 5);
};

struct CellDataTest : testing::Test
{
  std::shared_ptr <CellData> data = std::make_shared <CellData> (CellLocation{2, 3}, 13.0, true, false);
};

struct MatrixTest : testing::Test
{
//   std::vector <uint8_t> data {12, 23, 0, 1, 20, 4, 6, 7, 3, 9, 2, 7, 10, 15, 19, 3};
  std::vector <uint32_t> data {12, 23, 0, 1, 20, 4, 6, 7, 3, 9, 2, 7};
  std::shared_ptr < Matrix<uint32_t> > mat = std::make_shared < Matrix<uint32_t> > (4, 3, data); 
    // Matrix<uint8_t> mat {4, 4, data};
};

struct CostTest : testing::Test
{
    const double pi = double(3.1415926535897932385);
    std::shared_ptr < Cost > upHill = std::make_shared < UpHillCost > ();
    std::shared_ptr < Cost > downHill = std::make_shared < DownHillCost > ();
    CellLocation s{0, 0};
    CellLocation t{1, 1};
    uint8_t elevS = 1;
    uint8_t elevT = 3;
};

TEST_F(CellLocationTest, TestGet)
{
    EXPECT_TRUE(cell->X() == 10 && cell->Y() == 5);
}

TEST_F(CellLocationTest, TestSet)
{
    cell->setX(12);
    cell->setY(15);
    EXPECT_TRUE(cell->X() == 12 && cell->Y() == 15);
}

TEST_F(CellLocationTest, TestEQOperator)
{
    CellLocation t{10, 5};
    EXPECT_TRUE(*cell == t);
}

TEST_F(CellDataTest, TestGet)
{
    EXPECT_TRUE(data->getLoc().X() == 2 && data->getLoc().Y() == 3 && data->getWeight() == 13.0 && data->getVisited() == true && data->getPath() == false);
}

TEST_F(CellDataTest, TestSet)
{
    data->setLoc(14, 23);
    data->setWeight(11.0);
    data->setVisited(false);
    data->setPath(true);
    EXPECT_TRUE(data->getLoc().X() == 14 && data->getLoc().Y() == 23 && data->getWeight() == 11.0 && data->getVisited() == false && data->getPath() == true);
}

TEST_F(MatrixTest, TestOperator)
{
    EXPECT_TRUE(mat->operator()(2, 2) == 3 && mat->operator()(1, 1) == 20 && mat->operator()(3, 0) == 9);
}

TEST_F(MatrixTest, TestTotalSize)
{
    EXPECT_EQ(12, mat->getTotalSize());
}

TEST_F(MatrixTest, TestSizeRowCols)
{
    EXPECT_TRUE(mat->getRows() == 4 && mat->getCols() == 3);
}

TEST_F(MatrixTest, TestGetData)
{
    EXPECT_EQ(12, *mat->data());
}

TEST_F(CostTest, TestOctile)
{
    EXPECT_TRUE(upHill->computeCost(s, elevS, t, elevT, Cost::CostModel::Octile) == std::sqrt(2.0) &&
                downHill->computeCost(s, elevS, t, elevT, Cost::CostModel::Octile) == std::sqrt(2.0) );
}

TEST_F(CostTest, TestPeak)
{
    EXPECT_TRUE(upHill->computeCost(s, elevS, t, elevT, Cost::CostModel::Peak) == 3.0 &&
                downHill->computeCost(s, elevS, t, elevT, Cost::CostModel::Peak) == 3.0 );
}

TEST_F(CostTest, TestMeanPeak)
{
    EXPECT_TRUE(upHill->computeCost(s, elevS, t, elevT, Cost::CostModel::MeanPeak) == 2.0 &&
                downHill->computeCost(s, elevS, t, elevT, Cost::CostModel::MeanPeak) == 2.0 );
}

TEST_F(CostTest, TestL2)
{
    const double up = std::sqrt(6);
    const double down = 1.2 / std::sqrt(6);
    EXPECT_TRUE(upHill->computeCost(s, elevS, t, elevT, Cost::CostModel::L2) == up &&
                downHill->computeCost(s, elevS, t, elevT, Cost::CostModel::L2) == down );
}

TEST_F(CostTest, TestL1)
{
    const double up = 4;
    const double down = 0.5;
    EXPECT_TRUE(upHill->computeCost(s, elevS, t, elevT, Cost::CostModel::L1) == up &&
                downHill->computeCost(s, elevS, t, elevT, Cost::CostModel::L1) == down );
}

TEST_F(CostTest, TestLInf)
{
    const double up = 2;
    const double down = 0.5;
    EXPECT_TRUE(upHill->computeCost(s, elevS, t, elevT, Cost::CostModel::LInf) == up &&
                downHill->computeCost(s, elevS, t, elevT, Cost::CostModel::LInf) == down );
}

TEST_F(CostTest, TestAngle)
{
    const double up = (std::atan2(2, std::sqrt(2)) * 180 / pi) / 15 ;
    const double down = 0.25 * up;
    EXPECT_TRUE(upHill->computeCost(s, elevS, t, elevT, Cost::CostModel::Angle) == up &&
                downHill->computeCost(s, elevS, t, elevT, Cost::CostModel::Angle) == down );
    // EXPECT_EQ(upHill->computeCost(s, elevS, t, elevT, Cost::CostModel::Angle), up);
    // EXPECT_EQ(downHill->computeCost(s, elevS, t, elevT, Cost::CostModel::Angle), down);
}

TEST_F(CostTest, TestDifficulty)
{
    const double up = ((std::atan2(2, std::sqrt(2)) * 180 / pi) / 15) * std::sqrt(6); ;
    const double down = 0.25 * up;
    EXPECT_TRUE(upHill->computeCost(s, elevS, t, elevT, Cost::CostModel::DifficultyLevel) == up &&
                downHill->computeCost(s, elevS, t, elevT, Cost::CostModel::DifficultyLevel) == down );
}

int main(int argc, char** argv) 
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}