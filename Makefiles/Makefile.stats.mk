
stats:
	@echo "$(sep)Statistics"
	@echo
	@echo "These provide statistics about the data and the configuration."
	@echo
	@echo "  stats-easy_node  ?"
	@echo "  stats-easy_logs  ?"
	@echo "  stats-easy_algo  ?"

stats-easy_node:
	rosrun easy_node summary

stats-easy_logs:
	rosrun easy_logs summary

stats-easy_algo:
	rosrun easy_algo summary
