package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.PointForwardTask;
import frc.robot.autonomous.tasks.WaitTask;

public class TestMode extends AutoModeBase {

    @Override
    public void queueTasks() {
        queueTask(new ParallelTask(new DriveTrajectoryTask("pleasegodihope")));
        // queueTask(new DriveTrajectoryTask("pleasework"));
        queueTask(new PointForwardTask());
        queueTask(new WaitTask(10));
    }

    @Override
    public Pose2d getBlueStartingPosition() {
        return new Pose2d(1, 6, Rotation2d.fromDegrees(0));
    }
    
}