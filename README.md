# Rule Geometry Engine

面向运动目标与规则区域分析的四层 C++20 几何算子库：

1. **Geometry Types** — `Point2`, `BBox`, `Line`, `PolygonView`, `PolylineView`
2. **Spatial Operators** — IoU、Contains、Distance、Intersects、SideOfLine
3. **Track Trajectory Operators** — `TrackState`, velocity/heading、轨迹关系、TrackPool
4. **Rule Geometry Engine** — EnterROI、LeaveROI、CrossLine、Stay、Follow、Approach

## 设计原则

- 库内无 `static` 可变状态，核心算子不抛异常
- 几何输入使用非拥有的 `std::span` 视图
- 轨迹历史使用固定容量 `FixedRingBuffer`
- 多边形裁剪等临时存储由调用方提供的 `ScratchBuffer` 承担

## 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++-13
cmake --build build
ctest --test-dir build --output-on-failure
```

## 快速示例

```cpp
#include "rge/rge.hpp"

rge::ScratchBuffer scratch;
const rge::BBox box = rge::BBox::from_xywh(0, 0, 10, 10);
const rge::Point2 roi[] = {{0, 0}, {20, 0}, {20, 20}, {0, 20}};

const double ratio = rge::spatial::overlap_ratio(
    box, rge::PolygonView{roi}, scratch).value();
```

## 目录结构

```
include/rge/     公共头文件
src/             空间算子与几何实现
tests/           GoogleTest 单元测试
examples/        示例程序
```
