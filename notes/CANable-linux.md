## 1. Install CAN utilities.
```
sudo apt-get update
sudo apt-get install can-utils
```

## 2. Check connected USB port.
```
ls /dev/ttyACM*
```
* expected: ttyACM0

## 3. Check serial, idvendor, idproduct.
```
udevadm info -a /dev/ttyACM0
```
or
```
lsusb
sudo lsusb -d 16d0:117e - v | grep -i serial 
```

## 4. Create a .rules file.
```
KERNEL=="ttyACM*",ATTRS{serial}=="<serial>",MODE:="0777",SYMLINK+="<any symlink name>"  
`KERNEL=="ttyACM*",ATTRS{serial}=="1111333E5D5D",MODE:="0777",SYMLINK+="example_symlink"`
```

## 5. Move a .rules file.
```
sudo cp example.rules /etc/udev/rules.d/
```

## 6. Apply the .rules file.
```
sudo udevadm control --reload-rules
```
* Reconnection.

## 7. Create CAN interface with symlink.
```
sudo slcand -o -c -s5 /dev/<symlink> <any interface name>
`sudo slcand -o -c -s5 /dev/example_symlink can0`
```
 
## 8. Bring up the CAN interface.
```
sudo ifconfig *can0* up
```

## 9. Confirm the interface.
```
ifconfig
```

## 10. Test connection.

  #### 1. Terminal A - listen:
  ```
  sudo candump *can0*
  ```
 
  #### 2. Terminal B - send:
  ```
  sudo cansend *can0* 000#R
  ```

## 11. Confirm received data on Terminal A.
```
*can0* 000 [0] remote request 
```

## 12. If using virtual CAN,
```
sudo ip link add dev vcan0 type vcan  
sudo ifconfig vcan0 up  
ifconfig
```