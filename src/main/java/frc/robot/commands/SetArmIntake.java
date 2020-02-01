/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.IntakeControlState;

public class SetArmIntake extends Command {
  double m_setpoint;
  IntakeControlState m_state;
  public SetArmIntake(double setpoint, IntakeControlState state) {
    m_setpoint = setpoint;
    m_state = state;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.getArm().setArmSetpoint(m_setpoint);
    Robot.getIntake().setState(m_state);
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
