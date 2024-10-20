package frc.robot.autonomous.modes;

import frc.robot.AllianceHelpers;
import frc.robot.autonomous.tasks.AutoTargetTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.ShooterTask;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Intake.IntakePivotTarget;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterPivotTarget;
import frc.robot.subsystems.Shooter.ShooterSpeedTarget;

public class CenterTwoNoteMidringMode extends AutoModeBase {
  public void queueTasks() {
    // Note 1 (preload)
    queueTask(new ShooterTask(ShooterPivotTarget.SUBWOOFER, ShooterSpeedTarget.MAX));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
    queueTask(new WaitTask(RobotConstants.config.Auto.Timing.k_shootRevTime));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    queueTask(new WaitTask(RobotConstants.config.Auto.Timing.k_shootFeedTime));

    // Note 2 (MidRing)
    queueTask(new ParallelTask(
        new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE),
        new DriveTrajectoryTask("Shoot, MidRing")));
    queueTask(new ParallelTask(
        new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE),
        new AutoTargetTask(AllianceHelpers.getAllianceSpeakerPose3d())));
    queueTask(new WaitTask(RobotConstants.config.Auto.Timing.k_intakeBounceTime));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    queueTask(new WaitTask(RobotConstants.config.Auto.Timing.k_shootFeedTime));

    // Done
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
    queueTask(new DriveForwardTask(0, 0));
  }
}
