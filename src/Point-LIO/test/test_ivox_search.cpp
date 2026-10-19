#include <gtest/gtest.h>

#include <pcl/point_types.h>

#include "ivox/ivox3d.h"

namespace {

using Map = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, pcl::PointXYZINormal>;

pcl::PointXYZINormal point(float x, float y = 0.0f, float z = 0.0f)
{
  pcl::PointXYZINormal result{};
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

TEST(IVoxSearch, MissingNeighborhoodClearsPreviousScanCorrespondences)
{
  Map map(Map::Options{});
  map.AddPoints({point(1.0f), point(1.01f), point(1.02f), point(1.03f), point(1.04f)});
  Map::PointVector neighbors;

  ASSERT_TRUE(map.GetClosestPoint(point(1.0f), neighbors, 5));
  ASSERT_EQ(neighbors.size(), 5u);

  // The estimator reuses each scan index's output vector on the next scan.
  // A query outside the map must not leave five old points that pass its gate.
  EXPECT_FALSE(map.GetClosestPoint(point(100.0f), neighbors, 5));
  EXPECT_TRUE(neighbors.empty());

  ASSERT_TRUE(map.GetClosestPoint(point(1.0f), neighbors, 5));
  EXPECT_EQ(neighbors.size(), 5u);
}

TEST(IVoxSearch, RangeRejectionClearsPreviousCorrespondences)
{
  Map map(Map::Options{});
  map.AddPoints({point(1.0f)});
  Map::PointVector neighbors;

  ASSERT_TRUE(map.GetClosestPoint(point(1.0f), neighbors, 5));
  ASSERT_EQ(neighbors.size(), 1u);

  // The voxel exists, but no point is within the requested matching radius.
  EXPECT_FALSE(map.GetClosestPoint(point(1.05f), neighbors, 5, 0.01));
  EXPECT_TRUE(neighbors.empty());
}

}  // namespace
