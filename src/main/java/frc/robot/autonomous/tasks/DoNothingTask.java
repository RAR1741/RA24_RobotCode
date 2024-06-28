package frc.robot.autonomous.tasks;

import frc.robot.RobotTelemetry;

public class DoNothingTask extends Task {
  @Override
  public void prepare() {
    RobotTelemetry.print("Starting do nothing auto...");
  }

  @Override
  public void update() {
    log(true);

    RobotTelemetry.print("Do nothing auto complete");
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void done() {
    log(false);
  }
}
