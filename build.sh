. /opt/poky/3.1.3/environment-setup-aarch64-poky-linux
make clean
make
if [ ! -d "out" ]; then
    mkdir out
fi
mv modules.order Module.symvers *.ko *.mod .*.cmd *.mod.c  *.o out/

FILE=out/amg883x.ko
if [ -f "$FILE" ]; then
    scp $FILE jerryzheng@172.16.81.29:/Users/jerryzheng/Public/nfshome/home/root
fi

