#include <gtest/gtest.h>

#include <queue>
#include <vector>

#include "path_searching/dyn_a_star.h"

TEST(DynAStarOpenSet, LowerCostSnapshotPreemptsOlderEntry)
{
  GridNode improved;
  improved.state = GridNode::OPENSET;
  improved.gScore = 10.0;
  improved.fScore = 10.0;

  GridNode next_best;
  next_best.state = GridNode::OPENSET;
  next_best.gScore = 5.0;
  next_best.fScore = 5.0;

  std::priority_queue<NodeComparator::OpenSetNode,
                      std::vector<NodeComparator::OpenSetNode>,
                      NodeComparator>
    open_set;

  open_set.push(makeOpenSetNode(&improved));
  open_set.push(makeOpenSetNode(&next_best));

  improved.gScore = 1.0;
  improved.fScore = 1.0;
  open_set.push(makeOpenSetNode(&improved));

  ASSERT_FALSE(open_set.empty());
  EXPECT_EQ(&improved, open_set.top().node);
  EXPECT_DOUBLE_EQ(1.0, open_set.top().gScore);
  EXPECT_TRUE(isCurrentOpenSetNode(open_set.top()));
  open_set.pop();

  ASSERT_FALSE(open_set.empty());
  EXPECT_EQ(&next_best, open_set.top().node);
  EXPECT_DOUBLE_EQ(5.0, open_set.top().gScore);
  EXPECT_TRUE(isCurrentOpenSetNode(open_set.top()));
  open_set.pop();

  ASSERT_FALSE(open_set.empty());
  EXPECT_EQ(&improved, open_set.top().node);
  EXPECT_DOUBLE_EQ(10.0, open_set.top().gScore);
  EXPECT_FALSE(isCurrentOpenSetNode(open_set.top()));
}
