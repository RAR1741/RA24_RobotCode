package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePivotTarget;
import frc.robot.subsystems.Intake.IntakeState;

public class IntakeTask extends Task {
	private Intake m_intake = Intake.getInstance();

	private IntakePivotTarget m_pivotTarget;
	private IntakeState m_intakeState;

	public IntakeTask(IntakePivotTarget target, IntakeState state) {
		m_pivotTarget = target;
		m_intakeState = state;
	}

	@Override
	public void start() {
		m_intake.setPivotTarget(m_pivotTarget);
		m_intake.setIntakeState(m_intakeState);
	}

	@Override
	public void update() {
		m_intake.periodic();
	}

  @Override
  public void done() {
    DriverStation.reportWarning("Intake task finished", false);
  }

	@Override
	public boolean isFinished() {
    return m_intake.isAtPivotTarget();
	}
}
