package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.ShooterTask;
import frc.robot.constants.ApolloConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Intake.IntakePivotTarget;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterPivotTarget;
import frc.robot.subsystems.Shooter.ShooterSpeedTarget;

public class FunnyNoteMode extends AutoModeBase {
  public VisionConstants getVisionTargetConstants() {
    return ApolloConstants.Vision.funnyNoteVisionConstants;
  }

  public void queueTasks() {
    // Note 1 (preload)
    // queueShooterSpinUp();
    // queueShoot();

    queueTask(new ParallelTask(
        new ShooterTask(ShooterPivotTarget.SUBWOOFER, ShooterSpeedTarget.MAX),
        new IntakeTask(IntakePivotTarget.STOW, IntakeState.EJECT)));

    queueDriveAndIntake("Funny1");
    queueAutoTarget();
    queueShoot();

    queueDriveAndIntake("Funny2");
    queueAutoTarget();
    queueShoot();

    queueDriveAndIntake("Funny3");
    queueAutoTarget();
    queueShoot();

    // queueDriveAndIntake("Funny4");
    // queueAutoTarget();
    // queueShoot();

    // queueDriveAndIntake("Funny5");
    // queueAutoTarget();
    // queueShoot();

    queueEnd();
  }
}
