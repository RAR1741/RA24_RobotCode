package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class PointForwardTask extends Task {
  private SwerveDrive m_swerve;

  public PointForwardTask() {
    m_swerve = SwerveDrive.getInstance();
  }

  @Override
  public void start() {
    m_swerve.pointForward();
  }

  @Override
  public void update() {
    log(true);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void done() {
    log(false);

    DriverStation.reportWarning("Auto point forward done", false);
  }
}
