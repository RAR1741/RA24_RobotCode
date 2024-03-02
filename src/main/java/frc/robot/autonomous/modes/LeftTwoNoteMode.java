package frc.robot.autonomous.modes;

import frc.robot.Constants;
import frc.robot.autonomous.tasks.AutoTargetTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.ShooterTask;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.subsystems.Intake.IntakePivotTarget;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterPivotTarget;
import frc.robot.subsystems.Shooter.ShooterSpeedTarget;

public class LeftTwoNoteMode extends AutoModeBase {
  public void queueTasks() { // Set up left of subwoofer
    
    queueTask(new ShooterTask(ShooterPivotTarget.SUBWOOFER,
        ShooterSpeedTarget.MAX));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));

    queueTask(new ParallelTask(
      new AutoTargetTask(getAllianceSpeakerPose()),
      new WaitTask(Constants.Auto.Timing.k_shootRevTime)));

    queueTask(new ParallelTask(
      new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER),
      new WaitTask(Constants.Auto.Timing.k_shootFeedTime)));

    queueTask(new ParallelTask(
      new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE),
      new DriveTrajectoryTask("Left, BotRing")));

    queueTask(new AutoTargetTask(getAllianceSpeakerPose()));

    queueTask(new ParallelTask(
      new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER),
      new WaitTask(Constants.Auto.Timing.k_shootFeedTime)));

    queueTask(new DriveForwardTask(0, 0));
  }
}
