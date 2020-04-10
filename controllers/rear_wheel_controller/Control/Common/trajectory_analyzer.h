#ifndef TRAJECTORYANALYZER_H
#define TRAJECTORYANALYZER_H

#include <vector>
#include <stdlib.h>
#include "math.h"

#include "../../Utils/type_init.h"
#include "../../Math/vector_2d.h"
#include "../../VehicleState/GeometricTrack/geometric_track.h"
#include "../../Configure/Configs/vehicle_config.h"

typedef struct _TargetTrack
{
    Vector2d point;
    float    yaw;
    float    curvature;
    float    velocity;
}TargetTrack;

/**
 * @brief The TrackLinkList class：曲线跟踪目标曲线链表
 */
// class TrackLinkList : public LinkList<TargetTrack>{};

class TrajectoryAnalyzer
{
public:
    TrajectoryAnalyzer();

    void Init(std::vector<TargetTrack> *vec);

    /**
     * @brief CalculateNearestPointByPosition 基于位置信息计算目标曲线上最近的点
     * @param x 当前位置x轴坐标
     * @param y 当前位置y轴坐标
     * @return 返回最近目标点信息
     */
    TargetTrack CalculateNearestPointByPosition(const double x, const double y);

    /**
     * @brief DistanceToEnd 计算离最后目标曲线点的距离
     * @param x 当前位置x轴坐标
     * @param y 当前位置y轴坐标
     * @return 离最后点的距离值
     */
    double DistanceToEnd(const double x, const double y)const;
private:
    std::vector<TargetTrack> *_trajectory_points_vector;
    std::vector<TargetTrack>::iterator _current_it;
    uint16_t _target_point_index;
};

#endif // TRAJECTORYANALYZER_H
