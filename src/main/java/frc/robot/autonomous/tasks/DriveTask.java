package frc.robot.autonomous.tasks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotTelemetry;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveTask extends Task {
  private SwerveDrive m_swerve;
  private double m_targetDistance;
  private double m_xSpeed, m_ySpeed;
  private Pose2d m_startPose;

  private Timer m_runningTimer = new Timer();
  private double m_lastTime = 0;

  public DriveTask(double xSpeed, double ySpeed, double distance) {
    m_swerve = SwerveDrive.getInstance();
    m_targetDistance = distance;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    // m_speed = speed;
  }

  @Override
  public void start() {
    m_runningTimer.reset();
    m_runningTimer.start();

    m_startPose = m_swerve.getPose();
  }

  @Override
  public void update() {
    log(true);

    m_swerve.drive(m_xSpeed, m_ySpeed, 0, false);
  }

  @Override
  public void updateSim() {
    // This simulates the robot driving in the positive x direction
    if (!RobotBase.isReal()) {
      Pose2d currentPose = m_swerve.getPose();

      // Move "forward", based on the robot's current rotation
      double newX = currentPose.getX()
          + m_xSpeed * (m_runningTimer.get() - m_lastTime) * Math.cos(currentPose.getRotation().getRadians());
      double newY = currentPose.getY()
          + m_ySpeed * (m_runningTimer.get() - m_lastTime) * Math.sin(currentPose.getRotation().getRadians());

      Pose2d newPose = new Pose2d(
          newX,
          newY,
          currentPose.getRotation());

      m_swerve.setPose(newPose);
      m_lastTime = m_runningTimer.get();
    }
  }

  @Override
  public boolean isFinished() {
    Pose2d relativePose = m_startPose.relativeTo(m_swerve.getPose());
    return Math.hypot(relativePose.getX(), relativePose.getY()) >= m_targetDistance;
  }

  @Override
  public void done() {
    log(false);

    RobotTelemetry.print("Auto driving done");
    m_swerve.drive(0, 0, 0, true);
    m_swerve.pointModules(1, 0, 0, true);
  }
}
