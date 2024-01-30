package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;

public class TestMode extends AutoModeBase {

    @Override
    public void queueTasks() {
        queueTask(new DriveTrajectoryTask("pleasework", 0.1, 0.01));
    }

    @Override
    public Pose2d getRedStartingPosition() {
        return new Pose2d(1.4, 4.52, Rotation2d.fromDegrees(0));
    }
    
}
