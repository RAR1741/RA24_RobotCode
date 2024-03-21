package frc.robot.autonomous.modes;

public class CenterFourNote extends AutoModeBase {
  public void queueTasks() {
    // Note 1 (preload)
    queueShooterSpinUp();
    queueShoot();

    // Note 2 (MidRing)
    queueDriveAndIntake("Shoot, MidRing");
    queueAutoTarget();
    queueShoot();

    // Note 3 (PodiumRing)
    queueDriveAndIntake("MidRing, PodiumRing");
    queueAutoTarget();
    queueShoot();

    // Note 4 (BotRing)
    queueDriveAndIntake("PodiumRing, BotRing");
    queueAutoTarget();
    queueShoot();

    // Done
    queueDriveAndIntake("BotRing, Mid");

    queueEnd();
  }
}
