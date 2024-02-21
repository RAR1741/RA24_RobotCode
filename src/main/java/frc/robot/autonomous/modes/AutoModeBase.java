package frc.robot.autonomous.modes;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.Field;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
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

    // TODO: make this also search though the ParallelTasks and SequentialTasks
    // this will likely need to be a recursive function
    // Loop over the m_tasks and find the first DriveTrajectoryTask
    for (Task task : m_tasks) {
      if (task instanceof DriveTrajectoryTask) {
        // Set the starting pose to the starting pose of the first DriveTrajectoryTask
        startingPose = ((DriveTrajectoryTask) task).getStartingPose();
        break;
      }
    }

    // If there isn't one, default to something visible
    if (startingPose == null) {
      // Default to the center of the field
      startingPose = new Pose2d(Field.k_width / 2, Field.k_length / 2, new Rotation2d(0));
    }

    SwerveDrive m_swerve = SwerveDrive.getInstance();
    // TODO: Fix PP rotation again

    // Reset the gyro to the starting rotation
    // m_swerve.resetGyro();
    // m_swerve.setGyroAngleAdjustment(startingPose.getRotation().getDegrees());
    // m_swerve.get

    m_swerve.resetOdometry(startingPose);
  };

  public Pose3d getAllianceSpeakerPose() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return Field.k_blueSpeakerPose;
    } else {
      return Field.k_redSpeakerPose;
    }
  }
}
