package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.SequentialTask;
import frc.robot.autonomous.tasks.WaitTask;

public class TestMode2 extends AutoModeBase {
  @Override
  public void queueTasks() {
    // queueTask(new ParallelTask(new DriveTrajectoryTask("pleasework")));
    // queueTask(new DriveTrajectoryTask("pleasegodihope"));
    queueTask(new SequentialTask(
        new DriveTrajectoryTask("pleasework"),
        new DriveTrajectoryTask("pleasegodihope")));
    // queueTask(new PointForwardTask());
    queueTask(new WaitTask(10));
  }

  @Override
  public Pose2d getBlueStartingPosition() {
    return new Pose2d(1.4, 4.52, Rotation2d.fromDegrees(0));
  }

}
