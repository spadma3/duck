/*
 * Motor.h
 *
 * Created: 23/07/2018 15:21:26
 *  Author: Marcus
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

#define I2CPWMCH_PWMA 8
#define I2CPWMCH_AIN2 9
#define I2CPWMCH_AIN1 10
#define I2CPWMCH_BIN1 11
#define I2CPWMCH_BIN2 12
#define I2CPWMCH_PWMB 13

#define PORTINDEX_PWMA 3
#define PORTINDEX_AIN2 4
#define PORTINDEX_AIN1 3
#define PORTINDEX_BIN1 4
#define PORTINDEX_BIN2 6
#define PORTINDEX_PWMB 5

#define PWMOutput_PWMA OCR1B
#define PWMOutput_PWMB OCR1D

extern void init_motor_outputs( void );
extern void update_motor_values(void);


#endif /* MOTOR_H_ */