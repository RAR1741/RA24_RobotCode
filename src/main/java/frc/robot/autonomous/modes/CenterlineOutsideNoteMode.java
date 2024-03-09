package frc.robot.autonomous.modes;

import frc.robot.AllianceHelpers;
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

public class CenterlineOutsideNoteMode extends AutoModeBase {
  public void queueTasks() {
    // Note one (preload)
    queueTask(new ShooterTask(ShooterPivotTarget.PODIUM, ShooterSpeedTarget.MAX));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
    // queueTask(new DriveTrajectoryTask("Dummy"));
    queueTask(new ParallelTask( // Ensure shooter is revved before going on to feeding shooter
        new WaitTask(Constants.Auto.Timing.k_shootRevTime),
        new AutoTargetTask(AllianceHelpers.getAllianceSpeakerPose3d())));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    queueTask(new WaitTask(Constants.Auto.Timing.k_shootFeedTime));

    // Note two
    queueTask(new ParallelTask(
        new DriveTrajectoryTask("UntestedShotInTheDark1"),
        new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE)));
    queueTask(new ParallelTask(
        new DriveTrajectoryTask("UntestedShotInTheDark2"),
        new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE)));
    queueTask(new AutoTargetTask(AllianceHelpers.getAllianceSpeakerPose3d()));
    queueTask(new DriveForwardTask(0, 0));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));

    queueTask(new WaitTask(Constants.Auto.Timing.k_shootFeedTime));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
  }
}
