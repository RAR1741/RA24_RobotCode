package frc.robot.autonomous.modes;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.Field;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.SequentialTask;
import frc.robot.autonomous.tasks.Task;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public abstract class AutoModeBase {
  private ArrayList<Task> m_tasks;

  public AutoModeBase() {
    m_tasks = new ArrayList<>();
  }

  public Task getNextTask() {
    // Pop the first task off the list and return it
    try {
      return m_tasks.remove(0);
    } catch (IndexOutOfBoundsException ex) {
      return null;
    }
  }

  public void queueTask(Task task) {
    m_tasks.add(task);
  }

  public abstract void queueTasks();

  public void setStartingPose() {
    // Figure out the first PathPlanner path
    Pose2d startingPose = null;

    SwerveDrive m_swerve = SwerveDrive.getInstance();

    startingPose = getFirstDriveTask(m_tasks).getStartingPose();

    // If there isn't one, default to something visible
    if (startingPose == null) {
      // Default to the center of the field
      startingPose = new Pose2d(Field.k_width / 2, Field.k_length / 2, new Rotation2d(0));
    }

    // TODO: Fix PP rotation again

    // Reset the gyro to the starting rotation
    // m_swerve.resetGyro();
    // m_swerve.setGyroAngleAdjustment(startingPose.getRotation().getDegrees());

    m_swerve.resetOdometry(startingPose);

    // if (DriverStation.getAlliance().get() == Alliance.Red) {
    // m_swerve.setGyroAngleAdjustment(180);
    // } else {
    // m_swerve.setGyroAngleAdjustment(0);
    // }
  };

  public Pose3d getAllianceSpeakerPose() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return Field.k_blueSpeakerPose;
    } else {
      return Field.k_redSpeakerPose;
    }
  }

  private DriveTrajectoryTask getFirstDriveTask(List<Task> tasks) {
    Task[] taskArray = Arrays.copyOf(tasks.toArray(), tasks.toArray().length, Task[].class);
    return getFirstDriveTask(taskArray);
  }

  private DriveTrajectoryTask getFirstDriveTask(Task[] tasks) {
    for (Task task : tasks) {
      if (task instanceof DriveTrajectoryTask t) {
        // Set the starting pose to the starting pose of the first DriveTrajectoryTask
        return t;
      } else if (task instanceof ParallelTask t) {
        return getFirstDriveTask(t.getTasks());
      } else if (task instanceof SequentialTask t) {
        return getFirstDriveTask(t.getTasks());
      }
    }

    DriverStation.reportError("No DriveTrajectoryTask found in auto", true);
    return null;
  }
}
