#ifndef CURVATURE_H
#define CURVATURE_H

#include <vector>

#include "math.h"

#include "../../Math/vector_2d.h"
// #include "../../Utils/link_list.h"
#include "../../Configure/Configs/vehicle_config.h"
#include "../../Control/Common/trajectory_analyzer.h"

#define SAMPLE_STEP     	( 0.01f )	// 采样步长
#define COEFFICIENT_TLS 	( 1.2f  )	// 目标曲线曲率因子

class Curvature
{
public:
    Curvature();

    void Init();
    /**
     * @brief GenerateCurvaturePointSets:使用Vector容器存储目标点信息
     * @param vec 目标曲线数据集
     * @param type 曲线类型
     */
    void GenerateCurvaturePointSets(std::vector<TargetTrack> *vec,uint16_t type);

    /**
     * @brief CurvatureCalculate 根据Vector形式的目标曲线数据集，计算曲线的斜率和曲率
     * @param vec 待计算的数据集
     */
    void CurvatureCalculate(std::vector<TargetTrack> *vec);

private:
    // Node<TargetTrack>* track_node;
    // TrackLinkList    * track_list;
};

#endif // CURVATURE_H
