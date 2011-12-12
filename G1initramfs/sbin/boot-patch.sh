#!/system/bin/sh

busybox mount -t vfat /dev/block/mmcblk0p1 /mnt/sdcard
rm -rf /mnt/sdcard/.android_secure
busybox umount /mnt/sdcard

if /sbin/busybox [ ! -f /system/cfroot/release-14-JVK- ]; 
then
# Remount system RW
    mount -t rfs -o remount,rw /dev/block/stl9 /system

# Remove startup and shutdown sounds, to make some space
    toolbox rm /system/etc/PowerOn.snd
    toolbox rm /system/etc/PowerOn.wav
    toolbox rm /system/media/audio/ui/PowerOff.wav

# ensure /system/xbin exists
    toolbox mkdir /system/xbin
    toolbox chmod 755 /system/xbin

# su
    toolbox rm /system/bin/su
    toolbox rm /system/xbin/su
    toolbox cat /res/misc/su > /system/xbin/su
    toolbox chown 0.0 /system/xbin/su
    toolbox chmod 4755 /system/xbin/su

# Superuser
    toolbox rm /system/app/Superuser.apk
    toolbox rm /data/app/Superuser.apk
    toolbox cat /res/misc/Superuser.apk > /system/app/Superuser.apk
    toolbox chown 0.0 /system/app/Superuser.apk
    toolbox chmod 644 /system/app/Superuser.apk

# BusyBox
    /sbin/busybox find /system/xbin -type l -exec rm -f {} \;

    toolbox rm /system/bin/busybox
    toolbox ln -s /sbin/busybox /system/bin/busybox
    toolbox rm /system/xbin/busybox
    toolbox ln -s /sbin/busybox /system/xbin/busybox

# CWM Reboot
    toolbox rm /system/app/CWMReboot.apk
    toolbox cat /res/misc/CWMReboot.apk > /system/app/CWMReboot.apk
    toolbox chown 0.0 /system/app/CWMReboot.apk
    toolbox chmod 644 /system/app/CWMReboot.apk

# Once be enough
    toolbox mkdir /system/cfroot
    toolbox chmod 755 /system/cfroot
    toolbox rm /data/cfroot/*
    toolbox rmdir /data/cfroot
    toolbox rm /system/cfroot/*
    echo 1 > /system/cfroot/release-14-JVK- 

# Remount system RO
    mount -t rfs -o remount,ro /dev/block/stl9 /system
fi;

read sync < /data/sync_fifo
rm /data/sync_fifo
