#include "curvature.h"

Curvature::Curvature()
{
    Init();
}

void Curvature::Init()
{

}

void Curvature::GenerateCurvaturePointSets(std::vector<TargetTrack> *vec,uint16_t type)
{
    TargetTrack temp_track;
    float index_x,gain;
    index_x = 0.0;
    if(vec->size() > 0)
    {
        vec->clear();
    }

    switch(type)
    {
    case 1:// 三角函数曲线
        index_x = -35.0;
        while(index_x < 0.0)
        {
            temp_track.point.setX(index_x);
            temp_track.point.setY(15.0);
            vec->push_back(temp_track);
            index_x = index_x + SAMPLE_STEP;
        }
        while(index_x < (M_PI/COEFFICIENT_TLS))
        {

            temp_track.point.setX(index_x);
            temp_track.point.setY(15.0 + 5.0f*(cosf(COEFFICIENT_TLS*index_x) - 1.0));
            vec->push_back(temp_track);
            index_x = index_x + SAMPLE_STEP;
        }
        while(index_x < 50.0)
        {
            temp_track.point.setX(index_x);
            temp_track.point.setY(5.0);
            vec->push_back(temp_track);
            index_x = index_x + SAMPLE_STEP;
        }
        break;

    case 3:// 圆
//        index_x = 0.0;
//        gain = 5;
//        while(index_x < M_PI)
//        {
//            temp_track.point.setX(gain*cosf(index_x));
//            temp_track.point.setY(gain*sinf(index_x)-gain);
//            vec->push_back(temp_track);
//            index_x = index_x + SAMPLE_STEP;
//        }
        gain = 10;
        index_x = M_PI2;
        while(index_x > -M_PI)
        {
            temp_track.point.setX(gain*cosf(index_x));
            temp_track.point.setY(gain*sinf(index_x)-gain);
            vec->push_back(temp_track);
            index_x = index_x - SAMPLE_STEP;
        }
        index_x = 0;
        while(index_x < (M_PI + M_PI2))
        {
            temp_track.point.setX(gain*cosf(index_x)-2*gain);
            temp_track.point.setY(gain*sinf(index_x)-gain);
            vec->push_back(temp_track);
            index_x = index_x + SAMPLE_STEP;
        }
        break;

    default:
        break;
    }
    CurvatureCalculate(vec);
}

void Curvature::CurvatureCalculate(std::vector<TargetTrack> *vec)
{
    float first_derivative_x,first_derivative_y;
    float second_derivative_x,second_derivative_y;
    for(std::vector<TargetTrack>::iterator it = vec->begin();it != vec->end();it++)
    {
        if(it == vec->begin())
        {
            first_derivative_x = (it + 1)->point.getX() - it->point.getX();
            first_derivative_y = (it + 1)->point.getY() - it->point.getY();
            second_derivative_x = (it + 2)->point.getX() + it->point.getX() - 2.0f * (it + 1)->point.getX();
            second_derivative_y = (it + 2)->point.getY() + it->point.getY() - 2.0f * (it + 1)->point.getY();
        }
        else if(it == vec->end())
        {
            first_derivative_x = it->point.getX() - (it-1)->point.getX();
            first_derivative_y = it->point.getY() - (it-1)->point.getY();
            second_derivative_x = (it - 2)->point.getX() + it->point.getX() - 2.0f * (it - 1)->point.getX();
            second_derivative_y = (it - 2)->point.getY() + it->point.getY() - 2.0f * (it - 1)->point.getY();
        }
        else
        {
            first_derivative_x = (it + 1)->point.getX() - it->point.getX();
            first_derivative_y = (it + 1)->point.getY() - it->point.getY();
            second_derivative_x = (it + 1)->point.getX() + (it - 1)->point.getX() - 2.0f * it->point.getX();
            second_derivative_y = (it + 1)->point.getY() + (it - 1)->point.getY() - 2.0f * it->point.getY();
        }
        it->yaw = atan2f(first_derivative_y,first_derivative_x);

        it->curvature = (second_derivative_y*first_derivative_x - second_derivative_x*first_derivative_y)
                                           / powf(powf(first_derivative_x,2)+powf(first_derivative_y,2),1.5);
    }
}