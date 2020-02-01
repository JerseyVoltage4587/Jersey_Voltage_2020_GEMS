package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.util.DriveSignal;
import jaci.pathfinder.Trajectory;

public class FollowPath extends Command {
  Trajectory m_left, m_right;
  public FollowPath(Trajectory left, 
      Trajectory right) {
    requires(Robot.getDrive());
    m_left = left;
    m_right = right;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(Robot.getKillAuto()){return;}
    Robot.getDrive().setTrajectories(m_left, m_right);
    Robot.getDrive().startPath();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Robot.getKillAuto()){
      return true;
    }
    return Robot.getDrive().getState() == DriveControlState.OPEN_LOOP;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if(Robot.getKillAuto()){
      Robot.getDrive().setOpenLoop(DriveSignal.NEUTRAL);
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
