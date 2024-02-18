package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterPivotTarget;
import frc.robot.subsystems.Shooter.ShooterSpeedTarget;

public class ShooterTask extends Task {
	private Shooter m_shooter = Shooter.getInstance();

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
		m_shooter.periodic();
	}

	@Override
	public boolean isFinished() {
		boolean isAtTarget = m_shooter.isAtTarget(m_pivotTarget);

		return isAtTarget;
	}
}
