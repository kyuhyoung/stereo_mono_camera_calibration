#   -s_mm = chessboard grid side length in millimeters.
#   -w = # of horizontal grids
#   -h = # of vertical grids
#   -width = camera resolution width
#   -height = camera resolution height
#   -image_list = xml file name where image file names are listed.
#   -show = zero for not displaying, non-zero for display
#   -nr = short for no-rectification
#   -th_overlap = area raitio threshold for overlap check
#   -sec_int = seconds for the idle time for next capture
#   -dir_img = folder to save the captured images.    

#   TX1, zed
#./save_stereo_images_exe -cam=1 -s_mm=24.95 -w=10 -h=7 -width=672 -height=376 -image_list=data/stereo_calib_khchoi.xml -th_overlap=0.6 -sec_int=7 -dir_img=data

#   devbox, zed
#./save_stereo_images_exe -cam=0 -s_mm=24.95 -w=10 -h=7 -width=672 -height=376 -image_list=data/stereo_calib_khchoi.xml -th_overlap=0.6 -sec_int=7 -dir_img=data

#   TX1, ocams
./save_stereo_images_exe -ocam -cam=1 -s_mm=24.95 -w=10 -h=7 -width=640 -height=360 -image_list=data/stereo_calib_khchoi.xml -th_overlap=0.6 -sec_int=7 -dir_img=data

#   devbox ocams
#./save_stereo_images_exe -ocam -cam=0 -s_mm=24.95 -w=10 -h=7 -width=640 -height=360 -image_list=data/stereo_calib_khchoi.xml -th_overlap=0.6 -sec_int=7 -dir_img=data
