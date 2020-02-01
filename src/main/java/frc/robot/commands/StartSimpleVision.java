package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.util.DriveSignal;

public class StartSimpleVision extends Command {
  
  public StartSimpleVision() {
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(Robot.getKillAuto()){return;}
    Robot.getDrive().startVisionDrive();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.getDrive().getState() == DriveControlState.OPEN_LOOP;
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
