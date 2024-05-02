package frc.robot.autonomous.modes;

import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;

public class ShootNoDriveMode extends AutoModeBase {
  public VisionConstants getVisionTargetConstants() {
    return RobotConstants.config.Vision.fourNoteVisionConstants;
  }

  public void queueTasks() {
    queueShooterSpinUp();
    queueAutoTarget();
    queueShoot();

    queueEnd();
  }
}
