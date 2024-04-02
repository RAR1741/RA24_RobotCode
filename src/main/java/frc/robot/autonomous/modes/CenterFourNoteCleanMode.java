package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.SequentialTask;
import frc.robot.autonomous.tasks.WaitForTargetTask;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.constants.ApolloConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Intake.IntakePivotTarget;
import frc.robot.subsystems.Intake.IntakeState;

public class CenterFourNoteCleanMode extends AutoModeBase {
  public VisionConstants getVisionTargetConstants() {
    return ApolloConstants.Vision.fourNoteVisionConstants;
  }

  public void queueTasks() {
    // Note 1 (preload)
    queueShooterSpinUp();
    queueShoot();

    queueTask(new ParallelTask(
        new DriveTrajectoryTask("4NoteClean"),
        new SequentialTask(
            // Note 2 (PodiumRing)
            new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE, true),
            new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE),
            new WaitForTargetTask(),
            new ParallelTask(
                new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER),
                new WaitTask(ApolloConstants.Auto.Timing.k_shootFeedTime)),
            // Note 3 (MidRing)
            new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE, true),
            new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE),
            new WaitForTargetTask(),
            new ParallelTask(
                new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER),
                new WaitTask(ApolloConstants.Auto.Timing.k_shootFeedTime)),
            // Note 4 (BotRing)
            new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE, true),
            new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE),
            new WaitForTargetTask(),
            new ParallelTask(
                new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER),
                new WaitTask(ApolloConstants.Auto.Timing.k_shootFeedTime)))));

    // Done
    // queueDriveAndIntake("BotRing, Mid");

    queueEnd();
  }
}
