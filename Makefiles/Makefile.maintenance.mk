
maintenance:
	@echo "$(sep)Maintenance"
	@echo
	@echo 'A couple of utilities for robot maintenance.'
	@echo
	@echo '- `make maintenance-fix-time`:     Fixes the time.'
	@echo '- `make  maintenance-clean-pyc`:    Removes pyc files.'
	@echo
	@echo
	
maintenance-fix-time:
	echo 'Calling ntpdate to fix time'
	sudo ntpdate -u us.pool.ntp.org
	# sudo ntpdate -s time.nist.gov

maintenance-clean-pyc:
	@echo
	@echo Cleaning '*.pyc' files around
	find catkin_ws/ -name '*.pyc' -delete
