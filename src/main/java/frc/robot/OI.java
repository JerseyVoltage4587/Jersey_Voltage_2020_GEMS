/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.util.JoyButton;
import frc.robot.subsystems.Intake.IntakeControlState;
import frc.robot.commands.ClimbLevel2;
import frc.robot.commands.PlaceGamepiece;
import frc.robot.commands.ResetArmIntake;
import frc.robot.commands.SetArmIntake;
import frc.robot.commands.SetLiftSetpoint;
import frc.robot.commands.SetIntakeState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
//import utility.JoyButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	private static OI mInstance = null;
	private Joystick stick1;
	Button	  buttonA1, buttonB1, buttonX1, buttonY1, leftBumper1, rightBumper1,startButton1, leftStickButton1, rightStickButton1;
	JoyButton leftTrigger1, rightTrigger1;
	Joystick  stick2;
	Button	  buttonA2, buttonB2, buttonX2, buttonY2, leftBumper2, rightBumper2, leftStickButton2, rightStickButton2, startButton2;
	JoyButton leftTrigger2, rightTrigger2;
	Joystick  driverStation;
	Button    toggleSwitch0, toggleSwitch1, toggleSwitch2, toggleSwitch3, toggleSwitch4, tinesSwitch, visionSwitch;
	Button	  count0Button1, count0Button2, count1Button1, count1Button2, count2Button1, count2Button2, count3Button1, count3Button2;

	// Return the singleton OI object, creating it if necessary.
	// Creating the object is synchronized, just in case two threads end up calling simultaneously.
	public static OI getInstance()
	{
		if(mInstance == null) {
			synchronized ( OI.class ) {
				mInstance = new OI();
			}
		}
		return mInstance;
	}
	
	public OI()
	{
		//stick1 = new Joystick(1);
		stick1			= new Joystick(1);
    	buttonA1		= new JoystickButton(stick1, 1);
    	buttonB1		= new JoystickButton(stick1, 2);
    	buttonX1		= new JoystickButton(stick1, 3);
    	buttonY1		= new JoystickButton(stick1, 4);
    	leftBumper1 	= new JoystickButton(stick1, 5);
    	leftTrigger1	= new JoyButton(stick1, JoyButton.JoyDir.DOWN, 2);
    	rightBumper1	= new JoystickButton(stick1, 6);
    	rightTrigger1	= new JoyButton(stick1, JoyButton.JoyDir.DOWN, 3);
		startButton1	= new JoystickButton(stick1, 8);
		leftStickButton1= new JoystickButton(stick1, 9);
		rightStickButton1=new JoystickButton(stick1, 10);
    	
    	stick2			= new Joystick(2);
    	buttonA2		= new JoystickButton(stick2, 1);
    	buttonB2		= new JoystickButton(stick2, 2);
    	buttonX2		= new JoystickButton(stick2, 3);
    	buttonY2		= new JoystickButton(stick2, 4);
    	leftBumper2 	= new JoystickButton(stick2, 5);
    	leftTrigger2	= new JoyButton(stick2, JoyButton.JoyDir.DOWN, 2);
    	rightBumper2	= new JoystickButton(stick2, 6);
    	rightTrigger2	= new JoyButton(stick2, JoyButton.JoyDir.DOWN, 3);
		leftStickButton2= new JoystickButton(stick2, 9);
		rightStickButton2=new JoystickButton(stick2, 10);
		startButton2	= new JoystickButton(stick2, 8);
    	
    	driverStation   = new Joystick(0);
    	toggleSwitch0   = new JoystickButton(driverStation, 1);
    	toggleSwitch1   = new JoystickButton(driverStation, 2);
    	tinesSwitch   	= new JoystickButton(driverStation, 3);
    	visionSwitch 	= new JoystickButton(driverStation, 7);
    	// turn auto off button 14
    	count0Button1 	= new JoystickButton(driverStation, 11);
    	count0Button2 	= new JoystickButton(driverStation, 10);
    	count1Button1 	= new JoystickButton(driverStation, 13);
    	count1Button2 	= new JoystickButton(driverStation, 12);
    	count2Button1 	= new JoystickButton(driverStation, 15);
    	count2Button2 	= new JoystickButton(driverStation, 16);
    	count3Button1 	= new JoystickButton(driverStation, 9);
		count3Button2 	= new JoystickButton(driverStation, 8);
		
		/*
		* TRI OFFSEASON CONTROLS
		*/
		buttonA1.whenPressed(new ResetArmIntake());
		leftBumper1.whenPressed(new SetArmIntake(Constants.kArmIntakeBallDeg,IntakeControlState.INTAKE_BALL));
		//leftTrigger1.whenPressed(new SetClimbMotor(-0.5));
		//leftTrigger1.whenReleased(new SetClimbMotor(0.0));
		rightBumper1.whenPressed(new SetIntakeState(IntakeControlState.INTAKE_HATCH));
		rightBumper1.whenReleased(new ResetArmIntake());
		rightTrigger1.whenPressed(new PlaceGamepiece());
		rightTrigger1.whenReleased(new ResetArmIntake());
		
		//startButton1.whenPressed(new KillAuto()); //dont run auto at TRI???
		
		buttonA2.whenPressed(new SetLiftSetpoint(Constants.kLiftRocket1));
		buttonB2.whenPressed(new SetLiftSetpoint(Constants.kLiftRocket2));
		buttonY2.whenPressed(new SetLiftSetpoint(Constants.kLiftRocket3));
		//buttonX2.whenPressed(new SetLiftSetpoint(Constants.kLiftCargoShip));
		//rightBumper2.whenPressed(new ClimbUp());
		//rightTrigger2.whenPressed(new ClimbRest());
		leftBumper2.whenPressed(new ClimbLevel2());
		//leftTrigger2.whenPressed(new ClimbHalfOn());


		/*
		COMPETITON SEASON CONTROLS
		buttonA1.whenPressed(new SetArmIntake(Constants.kArmSoftStopLow,IntakeControlState.OFF));
		leftBumper1.whenPressed(new SetArmIntake(Constants.kArmIntakeBallDeg,IntakeControlState.INTAKE_BALL));
		leftTrigger1.whenPressed(new SetArmIntake(Constants.kArmHoldBallDeg,IntakeControlState.SHOOT_BALL));
		leftTrigger1.whenReleased(new SetArmIntake(Constants.kArmSoftStopLow,IntakeControlState.OFF));
		rightBumper1.whenPressed(new StartSimpleVision());
		rightBumper1.whenReleased(new StartOpenLoop());
		rightTrigger1.whenPressed(new SetClimbMotor(-0.5));
		rightTrigger1.whenReleased(new SetClimbMotor(0.0));
		leftStickButton1.whenPressed(new SetIntakeState(IntakeControlState.INTAKE_HATCH));
		rightStickButton1.whenPressed(new SetIntakeState(IntakeControlState.PLACE_HATCH));
		startButton1.whenPressed(new KillAuto());
		
		buttonA2.whenPressed(new ClimbUp());
		buttonB2.whenPressed(new ClimbHalfOn());
		buttonY2.whenPressed(new ClimbRest());
		buttonX2.whenPressed(new ClimbLevel2());
		rightTrigger2.whenPressed(new SetLiftSetpoint(Constants.kLiftCargoShip));
		leftTrigger2.whenPressed(new SetLiftSetpoint(0));
		rightStickButton2.whenPressed(new SetLiftSetpoint(Constants.kLiftRocket1));//lvl 1 rocket
		startButton2.whenPressed(new SetLiftSetpoint(Constants.kLiftRocket2));//lvl 2 rocket
		rightBumper2.whenPressed(new SetLiftSetpoint(Constants.kLiftRocket3));//lvl 3 rocket
		//leftBumper2.whenPressed(new HoldArmForDefense());
		//leftStickButton2.whenPressed(new ManualArm());


		visionSwitch.whenPressed(new SetVisionPipeline(1));
		visionSwitch.whenReleased(new SetVisionPipeline(0));
		*/
	}

	// Get the value of the "drive" stick.
	public double getDrive()
	{
		return -1 * stick1.getRawAxis(1);
	}

	// Get the value of the "turn" stick.
	public double getTurn()
	{
		return stick1.getRawAxis(4);
	}

	public double getDrive2(){
		return 0.25 * stick2.getRawAxis(1);
	}

	public double getIntake(){
		return stick1.getRawAxis(3);
	}
	
	public int getPOV() {
		return stick1.getPOV();
	}
}
