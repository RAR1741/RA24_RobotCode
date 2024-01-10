package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autonomous.tasks.BrakeTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.PointForwardTask;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DefaultMode extends AutoModeBase {
  SwerveDrive m_swerve = SwerveDrive.getInstance();
  @Override
  public Pose2d getRedStartingPosition() {
    return new Pose2d(14.655021228445234, 4.458172598636864, Rotation2d.fromDegrees(180));
  }

  public void queueTasks() {
    queueTask(new ParallelTask(
        new PointForwardTask(),
        new WaitTask(0.5)));

    queueTask(new ParallelTask(
        new DriveForwardTask(2.1, 1.0)));

    m_swerve.stop();

    System.out.println("Done driving? Please!!");

    queueTask(new BrakeTask(true));
  }
}
