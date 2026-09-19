#ifndef _COMMON_TYPES_H
#define _COMMON_TYPES_H

#include <Eigen/Eigen>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace freedom{
enum class DynamicLevel : uint8_t
{
    STATIC = 0,
    AGGRESSIVE_DYNAMIC = 1,
    MODERATE_DYNAMIC = 2,
    CONSERVATIVE_DYNAMIC = 3
};

enum Label
{
    LABEL_ERROR = 0,
    LABEL_STATIC = 9,
    LABEL_DYNAMIC = 251
};

typedef Eigen::Vector3f Pointf;
typedef Eigen::Vector3d Point;
typedef Eigen::Vector3d PointBias;
typedef Eigen::Vector3i Index;
typedef Eigen::Vector3i IndexBias;
typedef size_t LinearIndex;
typedef std::vector<Point,Eigen::aligned_allocator<Point>> Points;
typedef std::vector<Index,Eigen::aligned_allocator<Index>> Indices;
typedef std::vector<LinearIndex> LinearIndices;

class IndexHash
{
    public:
    std::size_t operator()(const Index& v) const{
        // 无符号乘法允许按 32 位回绕，避免地图索引较大时发生有符号溢出。
        return (static_cast<uint32_t>(v.x()) * uint32_t{73856093}) ^
               (static_cast<uint32_t>(v.y()) * uint32_t{19349663}) ^
               (static_cast<uint32_t>(v.z()) * uint32_t{83492791});
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef std::unordered_map<Index,LinearIndex,IndexHash> Index2LinearIndexMap;
typedef std::unordered_map<LinearIndex,LinearIndex> LinearIndex2LinearIndexMap;
}
#endif
