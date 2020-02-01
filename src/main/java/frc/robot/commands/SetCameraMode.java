package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.util.DriveSignal;

public class SetCameraMode extends Command {
  
  boolean m_isVisionMode;
  public SetCameraMode(boolean isVisionMode) {
    m_isVisionMode = isVisionMode;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
    int num=m_isVisionMode?0:1;
    limelightTable.getEntry("camMode").forceSetNumber(num);
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
