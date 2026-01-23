## ls
ls -a       * Include hidden.
ls -l       * Detail.
ls *.txt    * Only .txt

## rm
rm test.txt
rm -r test      * Directory 
rm -rf test     * Confirm

## cp
cp -r target_directory copy_directory

## mv
mv main.js node_modules /home/linux/hn/     * Move
mv index.js main.js                         * Rename

## file
touch example.txt
mkdir -p /home/linux/hn/project/toy

## chmod
chmod 755 file.sh       * Owner: rwx, Group: r-x, Others: r-x  
chmod +x file.sh        * Add execute permission  
chmod -x file.sh        * Remove execute permission  
chmod u+x file.sh       * Add execute for user (owner)  
chmod g-w file.sh       * Remove write for group  
chmod o=r file.sh       * Set others to read only  

## tar
```
tar -czvf fileName.tar.gz ./targetDirectory  
-c  * Create a new archive  
-z  * Compress using gzip  
-v  * Verbosely list files processed  
-f  * Specify archive file name  
```
```
tar -xzvf file.tar.gz  
-x  * Extract files from the archive  
-z  * Use gzip to decompress  
-v  * Verbosely list files processed  
-f  * Specify archive file name  
```

## scp
```bash
scp -r dist lit@192.168.1.251:~/
scp -r lit@192.168.1.251:~/dist ./dist
```

## arm-none-eabi-objcopy(elf -> bin)
sudo apt-get install binutils-arm-none-eabi
arm-none-eabi-objcopy -O binary firmware.elf firmware.bin

## usb
lsusb

## dfu
sudo dfu-util -l
sudo dfu-util -a 0 -s 0x08000000:leave -D firmware.bin -d 0483:df11 -S 335536663334

## disk
lsblk
sudo fdisk -l

## mount
sudo mount /dev/sdb1 /mnt/usb
cp -r /mnt/usb/ti ~/
sudo umount /mnt/usb

## service
```bash
sudo cp ./services/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl start example1 example2
sudo systemctl enable example1 example2
sudo systemctl status example1 example2
journalctl -u example1 -u example2 -f
```

```ini
[Unit]
Description=Example Startup Service
After=network.target

[Service]
WorkingDirectory=/home/user/ws	# base directory
User=user

ExecStart=/bin/bash /home/user/ws/scripts/start_example.sh

Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```