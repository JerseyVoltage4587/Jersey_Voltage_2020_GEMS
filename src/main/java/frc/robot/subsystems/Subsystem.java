package frc.robot.subsystems;

import frc.robot.loops.Looper;

public abstract class Subsystem extends edu.wpi.first.wpilibj.command.Subsystem{
    public void writeToLog() {
    };

    public abstract void outputToSmartDashboard();

    public abstract void stop();

    public abstract void zeroSensors();

    public abstract void registerEnabledLoops(Looper enabledLooper);
}