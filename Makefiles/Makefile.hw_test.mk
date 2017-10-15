

hw-test:
	@echo "$(sep)Hardware tests"
	@echo
	@echo 'To perform hardware tests:'
	@echo
	@echo '- `make hw-test-camera`     :       Testing Camera HW by taking a picture (smile!).'
	@echo '- `make hw-test-kinematics  :       Testing kinematics calibration'
	@echo '- `make hw-test-turn-right`:        Calibration right turn'
	@echo '- `make hw-test-turn-left`:         Calibrating left turn'
	@echo '- `make hw-test-turn-forward`:      Calibrating forward turn'
	@echo
	@echo

hw-test-camera:
	echo "Testing Camera HW by taking a picture (smile!)."
	raspistill -t 1000 -o test-camera.jpg

hw-test-led: check-environment
	@echo "Calibration blinking pattern"
	bash -c "source environment.sh; rosrun rgb_led blink test_all_1"

hw-test-kinematics: check-environment
	@echo "Testing Kinematics Calibration"
	bash -c "rostest indefinite_navigation calibrate_kinematics.test veh:=$(vehicle_name)"

hw-test-turn-right: check-environment
	@echo "Calibrating right turn"
	bash -c "rostest indefinite_navigation calibrate_turn.test veh:=$(vehicle_name) type:=right"

hw-test-turn-left: check-environment
	@echo "Calibrating left turn"
	bash -c "rostest indefinite_navigation calibrate_turn.test veh:=$(vehicle_name) type:=left"

hw-test-turn-forward: check-environment
	@echo "Calibrating forward turn"
	bash -c "rostest indefinite_navigation calibrate_turn.test veh:=$(vehicle_name) type:=forward"
