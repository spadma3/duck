duckietown_package=$(catkin_ws)/src/00-infrastructure/duckietown
machines=$(duckietown_package)/machines
cloud_db=$(catkin_ws)/src/00-infrastructure/easy_logs/cloud.yaml

build:
	@echo "$(sep)Building commands"
	@echo
	@echo "Commands to build the software."
	@echo
	@echo '- `make build-catkin`       :  Runs `catkin_make`.'
	@echo '- `make build-parallel` :  Runs `catkin_make`, with 4 threads.'
	@echo
	@echo '- `make build-machines`       :  Builds the machines file.'
	@echo '- `make build-machines-clean` :  Removes the machines file.'
	@echo
	@echo '- `make build-clean`          :  Clean everything.'

$(machines): build-machines

build-machines:
	rosrun duckietown create-machines-file


cloud-download: $(cloud_db)

$(cloud_db):
	wget -O $@ "https://www.dropbox.com/s/vdl1ej8fihggide/duckietown-cloud.yaml?dl=1"

build-machines-clean:
	@echo
	@echo Removing machines file.
	rm -f $(machines)

build-clean: \
	build-catkin-clean \
	build-machines-clean

build-catkin:
	catkin_make -C $(catkin_ws)

build-catkin-parallel:
	catkin_make -C $(catkin_ws) --make-args "-j4"

build-catkin-parallel-max:
	catkin_make -C $(catkin_ws) --make-args "-j"

build-catkin-clean:
	@echo
	@echo Removing the directory $(catkin_ws)/build
	rm -rf $(catkin_ws)/build


.PHONY: check-environment

check-environment:
	# Put here procedures to check the environment is valid
	#-./what-the-duck
