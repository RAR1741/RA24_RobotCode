package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePivotTarget;
import frc.robot.subsystems.Intake.IntakeState;

public class IntakeTask extends Task {
	private Intake m_intake;

	private IntakePivotTarget m_pivotTarget;
	private IntakeState m_intakeState;

	public IntakeTask(IntakePivotTarget target, IntakeState state) {
		m_intake = Intake.getInstance();

		m_pivotTarget = target;
		m_intakeState = state;
	}

	@Override
	public void start() {
		m_intake.setPivotTarget(m_pivotTarget);
		m_intake.setState(m_intakeState);
	}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		if (m_pivotTarget == IntakePivotTarget.INTAKE_PIVOT_NONE && m_intakeState == IntakeState.INTAKE_STATE_NONE) {
			return true;
		}

		boolean isAtPivotTarget = m_intake.isAtPivotTarget(m_pivotTarget);
		boolean isAtState = m_intake.isAtState(m_intakeState);

		return isAtPivotTarget && isAtState;
	}
}
