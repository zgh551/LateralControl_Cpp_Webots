#include "trajectory_analyzer.h"

TrajectoryAnalyzer::TrajectoryAnalyzer()
{
//    track_list = new TrackLinkList();
    _trajectory_points_vector = new std::vector<TargetTrack>;
}

void TrajectoryAnalyzer::Init(std::vector<TargetTrack> *vec)
{
    _trajectory_points_vector = vec;
    _target_point_index = 0;
    _current_it = _trajectory_points_vector->begin();
}

/**
 * @brief CalculateNearestPointByPosition 基于位置信息计算目标曲线上最近的点
 * @param x 当前位置x轴坐标
 * @param y 当前位置y轴坐标
 * @return 返回最近目标点信息
 */
TargetTrack TrajectoryAnalyzer::CalculateNearestPointByPosition(const double x, const double y)
{
    Vector2d target_point,current_point;
    // uint16_t start_id,end_id;
    double min_value,current_value;
    // start_id = 0.0 > _target_point_index - 10 ? 0 : _target_point_index - 10;
    // end_id   = _trajectory_points_vector->size() < _target_point_index + 20 ? _trajectory_points_vector->size() : _target_point_index + 20;
    target_point = _trajectory_points_vector->at(0).point;
    current_point = Vector2d(x,y);
    min_value = (target_point - current_point).Length();
    for(uint16_t i = 0;i < _trajectory_points_vector->size();i++)
    {
        current_value = (_trajectory_points_vector->at(i).point - current_point).Length();
        if(current_value < min_value)
        {
            _target_point_index = i;
            min_value = current_value;
        }
    }
    return _trajectory_points_vector->at(_target_point_index);
}

/**
 * @brief DistanceToEnd 计算离最后目标曲线点的距离
 * @param x 当前位置x轴坐标
 * @param y 当前位置y轴坐标
 * @return 离最后点的距离值
 */
double TrajectoryAnalyzer::DistanceToEnd(const double x, const double y)const
{
    return (_trajectory_points_vector->back().point - Vector2d(x,y)).Length();
}
