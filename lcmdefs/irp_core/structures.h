#pragma once

#include <cv.h>
#include <highgui.h>


#define LRF_1FRM_LENGTH			361
#define LRF_RES					0.5f
#define RAD2DEG		57.295779513f	//(180.0/M_PI)
#define DEG2RAD		0.01745329252f	//(M_PI/180.0)
#define IMG_W		640
#define IMG_H		480
#define LARGE_VALUE 1000000

typedef struct _fpoint3D
{
	float	x;
	float	y;
	float	z;
}fpoint3D;

typedef struct _fpoint3D_2ch
{
	fpoint3D p0;
	fpoint3D p1;
}fpoint3D_2ch;

typedef struct _RGB_uc
{
	unsigned char r;
	unsigned char g;
	unsigned char b;
}RGB_uc;

typedef struct _event_data
{
	int64 ts;
	int index;
	int sensor_type;
	int frameCounter;
	int fileSize;
	int filePos;
}event_data;

typedef struct _cam_data
{
	int64 ts;
	int   index;
	int   frameCounter;
	int   fileSize;
	int   filePos;
}cam_data;

typedef struct _lrf_data
{
	int64 ts;
	int range[LRF_1FRM_LENGTH];
	int	frameCounter;
	int fileSize;
	int filePos;
}lrf_data;

typedef struct _gps_data
{
	int64		ts;
	fpoint3D	pos;
	int			available;
	float		satellite_time;
	double		latitude;
	double		longitude;
	float		precision;
	float		satellite_elevation[12];
	float		satellite_azimuth[12];
	float		satellite_SNR[12];
	int			frameCounter;
	int         fileSize;
	int			filePos;

	float		pdop;
	float		hdop;
	float		vdop;
	float		err_dev_lat;
	float		err_dev_lon;
	float		err_dev_alt;


}gps_data;


typedef struct _imu_data
{
	int64   ts;
	fpoint3D acc;
	fpoint3D mag;
	fpoint3D gyr;
	fpoint3D ori;
	int	     frameCounter;
	int      fileSize;
	int		 filePos;
}imu_data;


typedef struct _wheelEncoder_data
{
	int64 ts;
	int   L;
	int	  R;
	int	  frameCounter;
	int   fileSize;
	int   filePos;
}wheelEncoder_data;

typedef struct _pose
{
	float	x;
	float	y;
	float	z;
	float	roll;
	float	pitch;
	float	yaw;
	int64	ts;
// 	CvMat*	roll_M;
// 	CvMat*	pitch_M;
// 	CvMat*	yaw_M;
// 	CvMat*	rotation_M; //
}pose;






