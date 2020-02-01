package frc.robot.commands;


import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *
 */
public class DelayTime extends Command {

	long m_startTime;
	long m_delayTime;
    public DelayTime(double delayTime) {
    	m_delayTime = (long)delayTime * 1000 * 1000 * 1000;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	m_startTime = System.nanoTime();
    }

    // Called repeatedly whe n this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if(Robot.getKillAuto()){return true;}
        return m_delayTime <= (System.nanoTime() - m_startTime);
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
