##IMPORTANT
ALL CHANGES TO THE CODE MUST BE DONE IN THIS FOLDER. IT IS CONFIGURED FOR OPENCV AND OPENCV WILL BREAK IF IT IS NOT DONE WITH THESE FILES.
EASYOPENCV: https://github.com/OpenFTC/EasyOpenCV

##Extra for the Control Hub:
Because EasyOpenCv depends on OpenCV-Repackaged, you will also need to copy libOpenCvAndroid453.so from the /doc/native_libs folder of that repo into the FIRST folder on the USB storage of the Robot Controller (i.e. connect the Robot Controller to your computer with a USB cable, put it into MTP mode, browse the contents of the file system, and drag 'n drop the file into the "FIRST" folder at the top level of the filesystem). Note that Control Hubs default to MTP mode and thus should be recognized immediately upon plugging it in. For Mac, you will either need to use the Android File Transfer program, or you can use the built-in file explorer side pane in Android Studio.
OpenCV-Repackaged: https://github.com/OpenFTC/OpenCV-Repackaged
file: https://github.com/OpenFTC/OpenCV-Repackaged/raw/9a4d3d4bc001feffb3767842fa2de0c38a98883a/doc/native_libs/armeabi-v7a/libOpenCvAndroid453.so