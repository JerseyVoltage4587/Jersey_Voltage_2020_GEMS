/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class RobotMap {
	//CAN ID's
	public static final int DRIVE_RIGHT_TALON = 2;
	public static final int DRIVE_RIGHT_VICTOR_1 = 21;
	public static final int DRIVE_RIGHT_TALON_SLAVE = 21;
	public static final int DRIVE_RIGHT_VICTOR_2 = 22;
	public static final int DRIVE_LEFT_TALON = 1;
	public static final int DRIVE_LEFT_VICTOR_1 = 11;
	public static final int DRIVE_LEFT_VICTOR_2 = 12;
	public static final int CLIMB_FRONT_TALON = 3;
	public static final int CLIMB_BACK_TALON = 4;
	public static final int CLIMB_DRIVE_VICTOR = 5;
	public static final int LIFT_TALON = 40;
	public static final int LIFT_VICTOR = 41;
	public static final int ARM_VICTOR = 31;
	public static final int INTAKE_TALON = 51;
	public static final int HATCH_TALON = 50;

	public static final int CANIFIER = 1;

	//PWM's
	//don't exist
	
	//Solenoid's
	public static final int FINGERS = 1;
	public static final int POKE = 0;
	public static final int INTAKE_BRAKE = 2;

	public static final boolean kPokeOut = true;
	public static final boolean kPokeIn = false;
	public static final boolean kFingersOpen = false;
	public static final boolean kFingersClosed = true;
	public static final boolean kBrakeOn = false;
	public static final boolean kBrakeOff = true;
	
	//DIO's
	public static final int FRONT_ARM_STOP = 2;
	public static final int BACK_ARM_STOP = 3;
	public static final int LIFT_ENC_A = 0;
	public static final int LIFT_ENC_B = 1;

	//PDP slots
	//don't care
}
