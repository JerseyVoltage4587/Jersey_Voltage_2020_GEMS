package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.util.DriveSignal;

public class StartVisionDrive extends Command {
  
  public StartVisionDrive() {
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      Robot.getDrive().setVisionPath();
      //.getDrive().startPath();
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
    //Robot.getDrive().setOpenLoop(DriveSignal.NEUTRAL);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
