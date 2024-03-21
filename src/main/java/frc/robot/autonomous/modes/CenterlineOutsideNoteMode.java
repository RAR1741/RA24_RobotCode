package frc.robot.autonomous.modes;

import frc.robot.AllianceHelpers;
import frc.robot.autonomous.tasks.AutoTargetTask;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.ShooterTask;
import frc.robot.subsystems.Intake.IntakePivotTarget;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterPivotTarget;
import frc.robot.subsystems.Shooter.ShooterSpeedTarget;

public class CenterlineOutsideNoteMode extends AutoModeBase {
  public void queueTasks() {
    // Note 1 (preload)
    queueTask(new ParallelTask(
        new ShooterTask(ShooterPivotTarget.SUBWOOFER, ShooterSpeedTarget.MAX),
        new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE),
        new AutoTargetTask(AllianceHelpers.getAllianceSpeakerPose3d())));

    queueShoot();

    // Note 2 (Center 4)
    queueDriveAndIntake("CenterFieldMiddle, Center 4");
    queueDrive("Center 4, Shoot Stage Right");
    queueAutoTarget();
    queueShoot();

    // Note 3 (Center 5)
    queueDriveAndIntake("Shoot Stage Right, Center 5");
    queueDrive("Center 5, Shoot Stage Right");
    queueAutoTarget();
    queueShoot();

    queueEnd();
  }
}
