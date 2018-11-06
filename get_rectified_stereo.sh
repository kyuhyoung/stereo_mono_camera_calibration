# command line arguments
# -int = path to the left/right intrinsic parameter file
# -ext = path to the extrinsic parameter file
# -alfa = [0 ~ 1 or -1]. The scale factor for undistortion and rectification. Check the following post for the effect ( http://support.eyedea.co.kr:8200/browse/VIS-8?focusedCommentId=19304&page=com.atlassian.jira.plugin.system.issuetabpanels:comment-tabpanel#comment-19304 )
# -post = postfix for the resulted yml file name. 
# -input = path to the xml file where the left and right images are listed. 
# -dir = the directory where the resulted rectifed images are supposed to be saved. 
# -sec = display interval in seconds. 

# for the case of image list
./get_rectified_stereo_exe -int=data/zed_672x376/intrinsics.yml -ext=data/zed_672x376/extrinsics.yml -post=alfa_1 -input=data/stereo_calib_khchoi.xml -dir=data/zed_672x376/ -sec=1 -alfa=1
# for the case of camera input of WVGA zed camera.
#./get_rectified_stereo_exe -int=data/zed_672x376/intrinsics.yml -ext=data/zed_672x376/extrinsics.yml -post=cam -dir=data -cam=1 -sec=3 -width=672 -height=376
