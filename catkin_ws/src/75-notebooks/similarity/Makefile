
all: similarity-brief similarity-long similarity-long_both

clean:
	rm -rf logs out

download:
	rosrun easy_logs download 20160119-duckietown4_1car_LC1_joystick_1.qa0

brief: download
	rosrun easy_logs thumbnails -o out/brief --od logs/brief --max_images=256  -c 'parmake' '20160119-duckietown4_1car_LC1_joystick_1.qa0/{105:110}'

long: download
	rosrun easy_logs thumbnails -o out/long --od logs/long --max_images=256 -c 'parmake' '20160119-duckietown4_1car_LC1_joystick_1.qa0/{100:}'

long2: download
	rosrun easy_logs thumbnails -o out/long2 --od logs/long2 --max_images=256	 -c 'parmake' '20160119-duckietown4_1car_LC1_joystick_1.qa0/{101:}'

long_both: long long2
	rm -rf logs/long_both
	mkdir -p logs/long_both
	ln -s $(CURDIR)/logs/long logs/long_both/long
	ln -s $(CURDIR)/logs/long2 logs/long_both/long2


similarity: similarity-brief similarity-long

similarity-brief: brief
	./similarity.py -o out/similarity-brief --dirname logs/brief

similarity-long: long
	./similarity.py -o out/similarity-long --dirname logs/long

similarity-long_both: long_both
	./similarity.py -o out/similarity-long_both --dirname logs/long_both
