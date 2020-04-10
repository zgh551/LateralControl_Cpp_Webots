/*
 * vector_2d.h
 *
 *  Created on: January 2 2019
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: vector_2d.h                         COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the vector property             					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 2 2019      Initial Version                  */
/*****************************************************************************/

#ifndef MATH_VECTOR_2D_H_
#define MATH_VECTOR_2D_H_

#include "math.h"

class Vector2d {
public:
	Vector2d();
	Vector2d(const float x,const float y) noexcept:_x(x),_y(y){}
	virtual ~Vector2d();

	//矢量模长
	float Length(void)const;
	//矢量模长的平方
	float LengthSquare(void)const;
	// 矢量角度
	float Angle(void)const;
	//求连个矢量之间的距离
	float DistanceTo(const Vector2d &other)const;

    // 向量叉积 axb
    float CrossProduct(const Vector2d&other) const;
    // 向量内积 a.b
    float InnerProduct(const Vector2d&other) const;

	//矢量旋转，旋转角度逆时针为正，顺时针为负
	Vector2d rotate(const float angle) const;
	//矢量正交点计算
	Vector2d Orthogonal(const float angle) const;

	//运算符重载
	Vector2d operator+(const Vector2d &other) const;
	Vector2d operator-(const Vector2d &other) const;
	Vector2d operator*(const double ratio) const;

	float getX();
	void  setX(float value);

	float getY();
	void  setY(float value);

protected:
	float _x;
	float _y;
};

#endif /* MATH_VECTOR_2D_H_ */
