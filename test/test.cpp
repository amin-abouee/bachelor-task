#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include "cell_info.hpp"
#include "matrix.hpp"

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


int main(int argc, char** argv) 
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}