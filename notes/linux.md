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
>> 7 rwx / 6 rw- / 5 r-x / 4 r-- / 0 ---

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