
CC = g++
CFLAGS  = -g -Wall -std=c++11
INCLUDES = -I include/

default: all

%.o: example/%.cpp
	mkdir -p build
	$(CC) $(CFLAGS) $(INCLUDES) -o build/$@ -c $^

%.o: src/%.cpp
	mkdir -p build
	$(CC) $(CFLAGS) $(INCLUDES) -o build/$@ -c $^

%.o: identification/%.cpp
	mkdir -p build
	$(CC) $(CFLAGS) $(INCLUDES) -o build/$@ -c $^

example: example.o ESPNOW_manager.o ESPNOW_types.o Link_manager.o ETHERNET_types.o master_board_interface.o motor.o motor_driver.o logger.o trajectoryGenerator.o identification_actuator_sine_position_pd.o
	mkdir -p bin
	$(CC) $(CFLAGS) -o bin/example $(addprefix build/,$^) -pthread

example_pd: example_pd.o ESPNOW_manager.o ESPNOW_types.o Link_manager.o ETHERNET_types.o master_board_interface.o motor.o motor_driver.o logger.o trajectoryGenerator.o
	mkdir -p bin
	$(CC) $(CFLAGS) -o bin/example_pd $(addprefix build/,$^) -pthread

example_pd_motor: example_pd_motor.o ESPNOW_manager.o ESPNOW_types.o Link_manager.o ETHERNET_types.o master_board_interface.o motor.o motor_driver.o logger.o trajectoryGenerator.o
	mkdir -p bin
	$(CC) $(CFLAGS) -o bin/example_pd_motor $(addprefix build/,$^) -pthread

identification_actuator_static: identification_actuator_static.o ESPNOW_manager.o ESPNOW_types.o Link_manager.o ETHERNET_types.o master_board_interface.o motor.o motor_driver.o logger.o trajectoryGenerator.o
	mkdir -p bin
	$(CC) $(CFLAGS) -o bin/identification_actuator_static $(addprefix build/,$^) -pthread

identification_actuator_sine_current: identification_actuator_sine_current.o ESPNOW_manager.o ESPNOW_types.o Link_manager.o ETHERNET_types.o master_board_interface.o motor.o motor_driver.o logger.o trajectoryGenerator.o
	mkdir -p bin
	$(CC) $(CFLAGS) -o bin/identification_actuator_sine_current $(addprefix build/,$^) -pthread

identification_actuator_sine_position_pd: identification_actuator_sine_position_pd.o ESPNOW_manager.o ESPNOW_types.o Link_manager.o ETHERNET_types.o master_board_interface.o motor.o motor_driver.o logger.o trajectoryGenerator.o
	mkdir -p bin
	$(CC) $(CFLAGS) -o bin/identification_actuator_sine_position_pd $(addprefix build/,$^) -pthread

identification_actuator_accelerate_torque: identification_actuator_accelerate_torque.o ESPNOW_manager.o ESPNOW_types.o Link_manager.o ETHERNET_types.o master_board_interface.o motor.o motor_driver.o logger.o trajectoryGenerator.o
	mkdir -p bin
	$(CC) $(CFLAGS) -o bin/identification_actuator_accelerate_torque $(addprefix build/,$^) -pthread

identification_actuator_accelerate_position_pd: identification_actuator_accelerate_position_pd.o ESPNOW_manager.o ESPNOW_types.o Link_manager.o ETHERNET_types.o master_board_interface.o motor.o motor_driver.o logger.o trajectoryGenerator.o
	mkdir -p bin
	$(CC) $(CFLAGS) -o bin/identification_actuator_accelerate_position_pd $(addprefix build/,$^) -pthread

identification_actuator_constant_velocity_pd: identification_actuator_constant_velocity_pd.o ESPNOW_manager.o ESPNOW_types.o Link_manager.o ETHERNET_types.o master_board_interface.o motor.o motor_driver.o logger.o trajectoryGenerator.o
	mkdir -p bin
	$(CC) $(CFLAGS) -o bin/identification_actuator_constant_velocity_pd $(addprefix build/,$^) -pthread

playground: playground.o ESPNOW_manager.o ESPNOW_types.o Link_manager.o ETHERNET_types.o master_board_interface.o motor.o motor_driver.o logger.o trajectoryGenerator.o
	mkdir -p bin
	$(CC) $(CFLAGS) -o bin/playground $(addprefix build/,$^) -pthread

display_actuator: display_actuator.o ESPNOW_manager.o ESPNOW_types.o Link_manager.o ETHERNET_types.o master_board_interface.o motor.o motor_driver.o logger.o trajectoryGenerator.o
	mkdir -p bin
	$(CC) $(CFLAGS) -o bin/display_actuator $(addprefix build/,$^) -pthread

all: clear clean example example_pd example_pd_motor identification_actuator_static identification_actuator_sine_current identification_actuator_accelerate_torque identification_actuator_accelerate_position_pd identification_actuator_constant_velocity_pd

play: clear clean playground

clean: 
	#$(RM) build/*
	$(RM) bin/*

clean log:
	$(RM) logs/*

clear:
	clear
