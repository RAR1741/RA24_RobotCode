package frc.robot.autonomous.modes;

import frc.robot.constants.ApolloConstants;
import frc.robot.constants.VisionConstants;

public class ShootNoDriveMode extends AutoModeBase {
  public VisionConstants getVisionTargetConstants() {
    return ApolloConstants.Vision.fourNoteVisionConstants;
  }

  public void queueTasks() {
    queueShooterSpinUp();
    queueAutoTarget();
    queueShoot();

    queueEnd();
  }
}
