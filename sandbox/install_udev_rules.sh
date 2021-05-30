#!/bin/bash
#Moves file containing udev rules for addressing sensors to appropriate folder

sudo cp 99-roboracing.rules /etc/udev/rules.d/
sudo service udev restart
exit
$SHELL