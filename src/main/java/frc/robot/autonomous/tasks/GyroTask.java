package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class GyroTask extends Task {
  SwerveDrive m_swerve = SwerveDrive.getInstance();

  /**
   * Resets the gyro yaw measurement
   */
  public GyroTask() {
    m_swerve.resetGyro();
  }

  /**
   * Sets the gyro angle adjustment value
   *
   * @param adjustment in degrees (range: -360 to 360)
   */
  public GyroTask(double adjustment) {
    m_swerve.setGyroAngleAdjustment(adjustment);
  }

  @Override
  public void start() {

  }

  @Override
  public void update() {
    log(true);
  }

  @Override
  public void done() {
    log(false);

    DriverStation.reportWarning("Gyro task finished", false);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
