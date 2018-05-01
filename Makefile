
vehicle_name=$(shell hostname)
catkin_ws=catkin_ws


all:
	@$(MAKE) -s stats
	@$(MAKE) -s test
	@$(MAKE) -s build
	@$(MAKE) -s docker
	@$(MAKE) -s generate
	@$(MAKE) -s demos
	@$(MAKE) -s hw-test
	@$(MAKE) -s maintenance
	@$(MAKE) -s openhouse
	@$(MAKE) -s log

sep="\\n\\n\#\#\# "


include Makefiles/Makefile.stats.mk
include Makefiles/Makefile.test.mk
include Makefiles/Makefile.build.mk
include Makefiles/Makefile.docker.mk
include Makefiles/Makefile.generate.mk
include Makefiles/Makefile.demos.mk
include Makefiles/Makefile.hw_test.mk
include Makefiles/Makefile.maintenance.mk
include Makefiles/Makefile.openhouse.mk
include Makefiles/Makefile.log.mk
