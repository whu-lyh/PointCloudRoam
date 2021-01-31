/*
	Copyright (c) 2020 WuHan University. All Rights Reserved
	First create: 2018/10/04
	Detail: point type
	Author:	zouxianghong
	Email:	xhzou_whu@foxmail.com

	Modified: 2020/06/13 10:27
	Detail: add PointXYZRGBINTF type to save all the information in the origin las
	Author: liyuhao
	Email:	yhaoli@whu.edu.cn

*/
#pragma once

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

//Point Type: x/y/z/r/g/b/intensity/number of return/GPS time/flighting edge
//typedef unsigned char      uint8_t;//c++
struct PointXYZRGBINTF
{
	PCL_ADD_POINT4D;
	PCL_ADD_RGB;
	PCL_ADD_INTENSITY;
	uint8_t num_of_returns;
	uint8_t return_number;
	double gps_time;
	uint8_t flighting_line_edge;
	uint8_t classification;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN16;

//Point Type: x/y/z/intensity/number of return/GPS time/flighting edge
struct PointXYZINTF
{
	PCL_ADD_POINT4D;
	PCL_ADD_INTENSITY;
	uint8_t num_of_returns;
	uint8_t return_number;
	double gps_time;
	uint8_t flighting_line_edge;
	uint8_t classification;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN16;

//Point Type: x/y/z/GPS time
struct PointXYZT
{
	PCL_ADD_POINT4D;
	double gps_time;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN16;

//POINT_CLOUD_REGISTER
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBINTF,
(float, x, x)
(float, y, y)
(float, z, z)
(uint8_t, r, r)
(uint8_t, g, g)
(uint8_t, b, b)
(float, intensity, intensity)
(uint8_t, num_of_returns, num_of_returns)
(uint8_t, return_number, return_number)
(double, gps_time, gps_time)
(uint8_t, flighting_line_edge, flighting_line_edge)
(uint8_t, classification, classification)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZINTF,
(float, x, x)
(float, y, y)
(float, z, z)
(float, intensity, intensity)
(uint8_t, num_of_returns, num_of_returns)
(uint8_t, return_number, return_number)
(double, gps_time, gps_time)
(uint8_t, flighting_line_edge, flighting_line_edge)
(uint8_t, classification, classification)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZT,
(float, x, x)
(float, y, y)
(float, z, z)
(double, gps_time, gps_time)
)
