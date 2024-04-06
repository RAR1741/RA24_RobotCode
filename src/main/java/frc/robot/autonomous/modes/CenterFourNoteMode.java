package frc.robot.autonomous.modes;

import frc.robot.constants.ApolloConstants;
import frc.robot.constants.VisionConstants;

public class CenterFourNoteMode extends AutoModeBase {
  public VisionConstants getVisionTargetConstants() {
    return ApolloConstants.Vision.fourNoteVisionConstants;
  }

  public void queueTasks() {
    // Note 1 (preload)
    queueShooterSpinUp();
    queueShoot();

    // Note 2 (MidRing)
    queueDriveAndIntake("Shoot, MidRing");
    queueAutoTarget();
    queueShoot();

    // Note 3 (PodiumRing)
    queueDriveAndIntake("MidRing, PodiumRing");
    queueAutoTarget();
    queueShoot();

    // Note 4 (BotRing)
    queueDriveAndIntake("PodiumRing, BotRing");
    queueAutoTarget();
    queueShoot();

    // Done
    queueDriveAndIntake("BotRing, Mid");

    queueEnd();
  }
}
