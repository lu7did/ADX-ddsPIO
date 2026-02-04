
#*--- Create FAT image

dd if=/dev/zero of=fat12_64k.img bs=512 count=128
#newfs_msdos -F 12 -S 512 -c 1 fat12_64k.img
#hdiutil attach fat12_64k.img
#xxd -i fat12_64k.img > fat12_64k.h


hdiutil attach -nomount fat12_64k.img
newfs_msdos -F 12 -S 512 -v ADXDISK -c 1 /dev/rdisk5
hdiutil detach /dev/disk5
hdiutil attach fat12_64k.img
hdiutil detach /dev/disk5

xxd -i fat12_64k.img > fat12_64k.h



