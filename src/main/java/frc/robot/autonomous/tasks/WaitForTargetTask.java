package frc.robot.autonomous.tasks;

import frc.robot.AllianceHelpers;
import frc.robot.RobotTelemetry;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class WaitForTargetTask extends Task {
  private SwerveDrive m_swerve;

  public WaitForTargetTask() {
    m_swerve = SwerveDrive.getInstance();
  }

  @Override
  public void start() {
  }

  @Override
  public void update() {
    m_swerve.setRotationTarget(AllianceHelpers.getAllianceSpeakerRotationTarget());
    log(true);
  }

  @Override
  public boolean isFinished() {
    return m_swerve.isAimedAtTarget();
  }

  @Override
  public void done() {
    log(false);

    RobotTelemetry.print("Wait for target done");
  }
}
