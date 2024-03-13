package frc.robot.autonomous.tasks;

import frc.robot.subsystems.drivetrain.SwerveDrive;

public class BrakeTask extends Task {
  private SwerveDrive m_swerve;
  private boolean m_brake;

  public BrakeTask(boolean brake) {
    m_brake = brake;
    m_swerve = SwerveDrive.getInstance();
  }

  @Override
  public void start() {
    m_swerve.setBrakeMode(m_brake);
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

    m_swerve.drive(0, 0, 0, false);
  }
}
