
stats:
	@echo "$(sep)Statistics"
	@echo
	@echo 'These provide statistics about the data and the configuration.'
	@echo
	@echo '- `make stats-easy_node`:  Prints summary of declared nots using the EasyNode frameworks.'
	@echo '- `make stats-easy_logs`:  Prints summary of available logs.'
	@echo '- `make stats-easy_algo`:  Prints summary of available algorithms.'
	@echo
	@echo
	
stats-easy_node:
	rosrun easy_node summary

stats-easy_logs:
	rosrun easy_logs summary

stats-easy_algo:
	rosrun easy_algo summary
