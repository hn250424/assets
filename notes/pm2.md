```
npm install -g pm2@latest  

pm2 start src/main.js --name <app name>  

<!-- First run 'pm2 startup' to get the system-specific command, then execute it -->
sudo su -c "env PATH=$PATH:/usr/bin pm2 startup -u <your-username> --hp /home/<your-username>"

pm2 save  

pm2 unstartup  

pm2 restart <app name>  
pm2 stop <app name>  
pm2 start <app name>  
pm2 delete <app name>  

pm2 list  
pm2 monit  
pm2 logs <app-name> --json --line 1000  

vi src/main.js  
pm2 restart <app name>  
```