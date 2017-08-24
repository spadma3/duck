
build:
	@echo "$(sep)Building commands"
	@echo
	@echo "Commands to build the software."
	@echo
	@echo "    build-machines         Builds the machines file."
	@echo "    build-machines-clean   Removes the machines file."
	@echo "    build-clean            Clean everything."

build-machines: $(machines)

$(machines): $(scuderia)
	rosrun duckietown create-machines-file


build-machines-clean:
	@echo
	@echo Removing machines file.
	rm -f $(machines)

build-clean: catkin-clean clean-machines


build-catkin:
	catkin_make -C $(catkin_ws)

build-catkin-parallel:
	catkin_make -C $(catkin_ws) --make-args "-j4"

build-catkin-clean: clean-pyc
	@echo
	@echo Removing the directory $(catkin_ws)/build
	rm -rf $(catkin_ws)/build


.PHONY: check-environment

check-environment:
	# Put here procedures to check the environment is valid
	#-./what-the-duck
