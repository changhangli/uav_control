@echo off
REM 把本目录拷到 Mini-Pc。先确保能 ssh 到 192.168.43.72
scp -r "%~dp0" uav@192.168.43.72:~/uav_worklog/scripts/listen_hover_test_upload
echo.
echo 登录 Mini-Pc 后执行:
echo   mkdir -p ~/uav_worklog/scripts
echo   rm -rf ~/uav_worklog/scripts/listen_hover_test
echo   mv ~/uav_worklog/scripts/listen_hover_test_upload ~/uav_worklog/scripts/listen_hover_test
echo   chmod +x ~/uav_worklog/scripts/listen_hover_test/*.sh
pause
