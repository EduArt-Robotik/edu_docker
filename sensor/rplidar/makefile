install-udev-rules:
	sudo cp rplidar.rules /etc/udev/rules.d
	sudo service udev reload
	sudo service udev restart

remove-udev-rules:
	sudo rm /etc/udev/rules.d/rplidar.rules
	sudo service udev reload
	sudo service udev restart
