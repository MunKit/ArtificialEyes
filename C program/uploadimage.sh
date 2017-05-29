#chmod +x uploadimage.sh
#capture image
ffmpeg -loglevel panic -y -s 320x240 -f video4linux2 -i /dev/video0 \
-vframes 1 ./Object.jpeg
#upload image
/home/my021usm/Downloads/Dropbox-Uploader/dropbox_uploader.sh upload \
Object.jpeg /Apps/IOT123/
