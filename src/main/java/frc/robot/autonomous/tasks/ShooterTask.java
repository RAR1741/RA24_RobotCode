package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterPivotTarget;

public class ShooterTask extends Task {
  private Shooter m_shooter;

  private ShooterPivotTarget m_pivotTarget;
	private double m_speed; // in rpm

  public ShooterTask(ShooterPivotTarget target, double speed) {
    m_pivotTarget = target;
    m_speed = speed;
  }

	@Override
	public void start() {
    m_shooter.setAngle(m_pivotTarget);
    m_shooter.setSpeed(m_speed);
	}

	@Override
	public void update() {
    m_shooter.periodic();
	}

	@Override
	public boolean isFinished() {
		boolean isAtTarget = m_shooter.isAtTarget(m_pivotTarget);

		return isAtTarget;
	}
}
