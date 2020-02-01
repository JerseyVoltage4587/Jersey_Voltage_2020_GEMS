package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.IntakeControlState;

public class Hab2Left2Hatch extends CommandGroup {
  
  public Hab2Left2Hatch() {
    addParallel(new SetIntakeStateAuto(IntakeControlState.INTAKE_HATCH));
    addParallel(new SetVisionPipeline(1));
    addSequential(new FollowPath(Robot.getPathManager().getTrajMap().get("hab2ToLeftNearCargoLeft"), Robot.getPathManager().getTrajMap().get("hab2ToLeftNearCargoRight")));
    addSequential(new StartSimpleVisionAuto());
    addSequential(new SetIntakeStateAuto(IntakeControlState.PLACE_HATCH));
    addSequential(new DelayTime(0.1));
    addParallel(new SetVisionPipeline(0));
    addSequential(new FollowPath(Robot.getPathManager().getTrajMap().get("leftNearCargoToLoadingLeft"), Robot.getPathManager().getTrajMap().get("leftNearCargoToLoadingRight")));
    addSequential(new SetIntakeStateAuto(IntakeControlState.INTAKE_HATCH));
    addSequential(new StartSimpleVisionAuto());
    /*addSequential(new DelayTime(0.5));
    addSequential(new FollowPath(Robot.getPathManager().getTrajMap().get("loadingToLeftMiddleCargoLeft"), Robot.getPathManager().getTrajMap().get("loadingToLeftMiddleCargoRight")));
    addSequential(new StartSimpleVision());
    addSequential(new SetIntakeState(IntakeControlState.PLACE_HATCH));
    */
  }
}
