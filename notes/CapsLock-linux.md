1. Create and add a key mapping file.
#### vim .Xmodmap
```
keycode 91 = Caps_Lock          * 91 == Any unused key  
keycode 66 = Pointer_Button1    * 66 == Caps_Lock
```

2. Insatall xkbset.
#### sudo apt-get install -y xkbset  

3. Create and add a startup script file for booting.
#### vim capsLock.sh
```
\#!/bin/bash  
export DISPLAY=:0  
/usr/bin/xmodmap /home/user/.Xmodmap  
sleep 5  
/usr/bin/xkbset mousekeys  
```

4. Grant execute permission to the script. 
#### chmod +x capsLock.sh

5. Register services.
#### cd /etc/systemd/system
#### vim capsLockService.service
```
[Unit]  
Description=CapsLock key mapping service  
After=multi-user.target  
  
[Service]  
ExecStart=/home/hn/capsLock.sh  
Restart=on-failure  
Environment="DISPLAY=:0"  
User=hn  
  
[Install]  
WantedBy=multi-user.target
```

6. Restart demon.
#### sudo systemctl daemon-reload

7. Set the script to run at boot.
#### sudo systemctl enable capsLockService

8. Reboot.
#### sudo reboot



