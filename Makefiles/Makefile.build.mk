duckietown_package=$(catkin_ws)/src/00-infrastructure/duckietown
machines=$(duckietown_package)/machines
cloud_db=$(catkin_ws)/src/00-infrastructure/easy_logs/cloud.yaml

build:
	@echo "$(sep)Building commands"
	@echo
	@echo "Commands to build the software."
	@echo
	@echo '- `make build-catkin`              :  Runs `catkin_make`.'
	@echo '- `make build-catkin-parallel`     :  Runs `catkin_make`, with 4 threads.'
	@echo '- `make build-catkin-parallel-max` :  Runs `catkin_make`, with many threads.'
	@echo
	@echo '- `make build-machines`            :  Builds the machines file.'
	@echo '- `make build-machines-clean`      :  Removes the machines file.'
	@echo
	@echo '- `make build-clean`               :  Clean everything.'
	@echo
	@echo

$(machines): build-machines

build-machines:
	rosrun duckieteam create-machines


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
	catkin_make -C $(catkin_ws) --make-args --no-print-directory

build-catkin-parallel:
	catkin_make -C $(catkin_ws) --make-args --no-print-directory --jobs 4

build-catkin-parallel-max:
	catkin_make -C $(catkin_ws) --make-args --no-print-directory --jobs

build-catkin-clean:
	@echo
	@echo Removing the directory $(catkin_ws)/build
	rm -rf $(catkin_ws)/build
	@echo Removing the directory $(catkin_ws)/devel
	rm -rf $(catkin_ws)/devel


.PHONY: check-environment

check-environment:
	# Put here procedures to check the environment is valid
	#-./what-the-duck

pdoc_packages=\
	duckietown_utils\
	easy_logs\
	easy_algo\
	easy_node\
	easy_regression

pdoc_out=out-pdoc

pdoc:
	pdoc --overwrite --html --html-dir $(pdoc_out) duckietown_utils
	pdoc --overwrite --html --html-dir $(pdoc_out) easy_logs
	pdoc --overwrite --html --html-dir $(pdoc_out) easy_algo
	pdoc --overwrite --html --html-dir $(pdoc_out) easy_node
	pdoc --overwrite --html --html-dir $(pdoc_out) easy_regression

pdoc-clean:
	rm -rf  $(pdoc_out)
