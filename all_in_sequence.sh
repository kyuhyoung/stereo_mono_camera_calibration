#!/bin/sh

set -e

#if [ 1 -eq 0 ]; then

IS_OCAM=0

#CAM_IDX=0
CAM_IDX=1

ZED=zed
OCAMS=ocams

#CAMERA=$ZED                                                                                  
CAMERA=$OCAMS  
#CAMERA=logitech_c920  

if [ $CAMERA = $ZED ]; then 
	WIDTH=672
	HEIGHT=376
elif [ $CAMERA = $OCAMS ]; then
	WIDTH=640
	HEIGHT=360
	IS_OCAM=1
else 
	WIDTH=640
	HEIGHT=480
fi
echo "width : $WIDTH , height : $HEIGHT"      
echo "IS_OCAM : $IS_OCAM"   

#fi #[ 1 -eq 0 ]

SQUARE_MM=24.95
WIDTH_CHESS=10    
HEIGHT_CHESS=7
OBJECT=clock
ALPHA=0
SHRINK=0.5

CAM_ENV="$CAMERA"_"$WIDTH"x"$HEIGHT"
echo "CAM_ENV : $CAM_ENV"

CALIB_ENV="$CAM_ENV"_"$WIDTH_CHESS"x"$HEIGHT_CHESS"_"$SQUARE_MM"
echo "CALIB_ENV : $CALIB_ENV"

DIR_STEREO=~/work/eyedea/stereo_camera_calibration/master
#DIR_STEREO=/mnt/d/work/eyedea/stereo_camera_calibration/master
echo "DIR_STEREO : $DIR_STEREO"
[ ! -d "$DIR_STEREO" ] && echo "Error: directory '$DIR_STEREO' does NOT exist." && exit
DIR_CALIB="$DIR_STEREO"/data/"$CALIB_ENV"
echo "DIR_CALIB : $DIR_CALIB"
[ ! -d "$DIR_CALIB" ] && echo "Error: directory '$DIR_CALIB' does NOT exist." && exit

FILE_MONO="$DIR_CALIB"/cam_param_"$CALIB_ENV".yml
echo "FILE_MONO : $FILE_MONO"

FILE_INTR="$DIR_CALIB"/intrinsics_"$CALIB_ENV".yml
echo "FILE_INTR : $FILE_INTR"

FILE_EXTR="$DIR_CALIB"/extrinsics_"$CALIB_ENV".yml
echo "FILE_EXTR : $FILE_EXTR"

FILE_IMG_LIST="$DIR_CALIB"/stereo_image_list_$CALIB_ENV.xml
echo "FILE_IMG_LIST : $FILE_IMG_LIST"
[ ! -f "$FILE_IMG_LIST" ] && echo "$FILE_IMG_LIST does NOT exist." && exit

# 모노 카메라 이미지 저장
echo "save_stereo_images starts."
#$DIR_STEREO/save_stereo_images_exe -ocam=$IS_OCAM -cam=$CAM_IDX -mono -s_mm=$SQUARE_MM -w=$WIDTH_CHESS -h=$HEIGHT_CHESS -width=$WIDTH -height=$HEIGHT -image_list=$FILE_IMG_LIST -dir_img=$DIR_CALIB -th_overlap=0.6 -sec_int=7
echo "save_stereo_images finishes."

# 스테레오 카메라 이미지 저장
echo "save_stereo_images starts."
$DIR_STEREO/save_stereo_images_exe -ocam=$IS_OCAM -cam=$CAM_IDX -s_mm=$SQUARE_MM -w=$WIDTH_CHESS -h=$HEIGHT_CHESS -width=$WIDTH -height=$HEIGHT -image_list=$FILE_IMG_LIST -dir_img=$DIR_CALIB -th_overlap=0.6 -sec_int=7
echo "save_stereo_images finishes."

echo "stereo_calib_eyedea starts."
# 스테레오 카메라 캘리브레이션
#$DIR_STEREO/stereo_calib_eyedea_exe -alfa=$ALPHA -s=$SQUARE_MM -w=$WIDTH_CHESS -h=$HEIGHT_CHESS -dir_img=$DIR_CALIB -dir_calib=$DIR_CALIB -input=$FILE_IMG_LIST -postfix=$CALIB_ENV                                   
# 모노 카메라 캘리브레이션
#$DIR_STEREO/stereo_calib_eyedea_exe -mono -alfa=$ALPHA -s=$SQUARE_MM -w=$WIDTH_CHESS -h=$HEIGHT_CHESS -dir_img=$DIR_CALIB -dir_calib=$DIR_CALIB -input=$FILE_IMG_LIST -postfix=$CALIB_ENV                                   
echo "stereo_calib_eyedea finishes."

echo "get_rectified_stereo starts."
#   image file version 
#$DIR_STEREO/get_rectified_stereo_exe -input=$FILE_IMG_LIST -int=$FILE_INTR -ext=$FILE_EXTR -alfa=$ALPHA -post=alfa_$ALPHA -sec=1 -dir_img=$DIR_CALIB -dir_rect=$DIR_CALIB/rectified_result                                                                    
#$DIR_STEREO/get_rectified_stereo_exe -mono -input=$FILE_IMG_LIST -calib=$FILE_MONO -alfa=$ALPHA -post=alfa_$ALPHA -sec=10 -dir_img=$DIR_CALIB -dir_rect=$DIR_CALIB/rectified_result                                                                    
#   camera version
#$DIR_STEREO/get_rectified_stereo_exe -ocam=$IS_OCAM -mono -cam=$CAM_IDX -width=$WIDTH -height=$HEIGHT -int=$FILE_INTR -ext=$FILE_EXTR -alfa=$ALPHA -post=alfa_$ALPHA -sec=1 -dir_rect=$DIR_CALIB/rectified_result
echo "get_rectified_stereo finishes."
