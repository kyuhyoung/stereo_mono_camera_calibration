#   -s = length of the side of the chessboard square in millimeters. 
#   -w = # of grid in horizontal side. 
#   -h = # of grid in vertical side.  
#   -e = list of one-based indices to skip.
#   -dir = folder where the actual image files are.
#   -input = path to the xml file in which image file names are listed.
./stereo_calib_eyedea_exe -s=24.95 -w=10 -h=7 -dir=data/zed_672x376/ -e=10,8 -input=data/stereo_calib_khchoi.xml
