package frc.robot.autonomous.tasks;

public class DoNothingTask extends Task {
  @Override
  public void start() {
    System.out.println("Starting do nothing auto...");
  }

  @Override
  public void update() {
    log(true);

    System.out.println("Do nothing auto complete");
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
