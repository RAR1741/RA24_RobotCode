package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotTelemetry;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterPivotTarget;
import frc.robot.subsystems.Shooter.ShooterSpeedTarget;

public class ShooterTask extends Task {
  private Shooter m_shooter = Shooter.getInstance();
  // private boolean m_isSimFinished = false;

  private ShooterPivotTarget m_pivotTarget;
  private ShooterSpeedTarget m_speedTarget;
  private double m_speed; // in rpm

  public ShooterTask(ShooterPivotTarget target, double speed) {
    m_pivotTarget = target;
    m_speed = speed;
  }

  public ShooterTask(ShooterPivotTarget target, ShooterSpeedTarget speed) {
    m_pivotTarget = target;
    m_speedTarget = speed;
  }

  @Override
  public void start() {
    RobotTelemetry.print("Auto shooter start");

    m_shooter.setAngle(m_pivotTarget);
    if (Double.isNaN(m_speed)) {
      m_shooter.setSpeed(m_speed);
    }
    if (m_speedTarget != null) {
      m_shooter.setSpeed(m_speedTarget);
    }
  }

  @Override
  public void update() {
    log(true);

    // m_shooter.periodic();
  }

  @Override
  public void updateSim() {
    // I chose not to delete this :P
    // if (!RobotBase.isReal()) {
    // m_shooter.periodic();
    // m_isSimFinished = m_shooter.isAtTarget(m_pivotTarget);
    // }
  }

  @Override
  public boolean isFinished() {
    boolean isAtTarget = m_shooter.isAtTarget() && m_shooter.isAtSpeed();

    return isAtTarget || !RobotBase.isReal();
  }

  @Override
  public void done() {
    log(false);

    RobotTelemetry.print("Auto shooter done");
  }
}
