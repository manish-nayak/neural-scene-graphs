@echo off

set DATADIR=.\data\vkitti2
mkdir %DATADIR%

powershell -Command "Invoke-WebRequest -Uri http://download.europe.naverlabs.com//virtual_kitti_2.0.3/vkitti_2.0.3_rgb.tar -OutFile vkitti_2.0.3_rgb.tar"
powershell -Command "tar -xf vkitti_2.0.3_rgb.tar -C %DATADIR%"

powershell -Command "Invoke-WebRequest -Uri http://download.europe.naverlabs.com//virtual_kitti_2.0.3/vkitti_2.0.3_textgt.tar.gz -OutFile vkitti_2.0.3_textgt.tar.gz"
powershell -Command "tar -xf vkitti_2.0.3_textgt.tar.gz -C %DATADIR%"

del vkitti_2.0.3_rgb.tar
del vkitti_2.0.3_textgt.tar.gz