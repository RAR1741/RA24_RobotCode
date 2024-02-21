package frc.robot.autonomous.modes;

import frc.robot.Constants;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ShooterTask;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.subsystems.Intake.IntakePivotTarget;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterPivotTarget;
import frc.robot.subsystems.Shooter.ShooterSpeedTarget;

public class FourNoteMode extends AutoModeBase {
  public void queueTasks() {
    // Note 1 (preload)
    queueTask(new ShooterTask(ShooterPivotTarget.SPEAKER, ShooterSpeedTarget.MAX));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
    queueTask(new WaitTask(Constants.Auto.Timing.k_shootRevTime));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    queueTask(new WaitTask(Constants.Auto.Timing.k_shootFeedTime));

    // Note 2 (center)
    queueTask(new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE));
    queueTask(new DriveTrajectoryTask("Shoot, MidRing"));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
    queueTask(new DriveTrajectoryTask("MidRing, Shoot"));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    queueTask(new WaitTask(Constants.Auto.Timing.k_shootFeedTime));
    queueTask(new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE));

    // Note 3 (not shot, from center line)
    queueTask(new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE));
    queueTask(new DriveTrajectoryTask("Shoot, PodiumRing"));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
    queueTask(new DriveTrajectoryTask("PodiumRing, Shoot"));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    queueTask(new WaitTask(Constants.Auto.Timing.k_shootFeedTime));
    queueTask(new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE));

    // Note 4 (not shot, from center line)
    queueTask(new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE));
    queueTask(new DriveTrajectoryTask("Shoot, BotRing"));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
    queueTask(new DriveTrajectoryTask("BotRing, Shoot"));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    queueTask(new WaitTask(Constants.Auto.Timing.k_shootFeedTime));
    queueTask(new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE));

    // Done
    queueTask(new ShooterTask(ShooterPivotTarget.SPEAKER, ShooterSpeedTarget.OFF));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
  }
}
