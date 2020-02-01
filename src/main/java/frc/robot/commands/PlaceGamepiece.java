/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeControlState;

public class PlaceGamepiece extends Command {
  
  public PlaceGamepiece() {

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(Robot.getIntake().getHasHatch()){
      Robot.getIntake().setState(IntakeControlState.PLACE_HATCH);
    }else{
      Robot.getIntake().setState(IntakeControlState.SHOOT_BALL);
      Robot.getArm().setArmSetpoint(Constants.kArmHoldBallDeg);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
