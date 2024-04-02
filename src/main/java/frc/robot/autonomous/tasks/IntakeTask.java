package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePivotTarget;
import frc.robot.subsystems.Intake.IntakeState;

public class IntakeTask extends Task {
  private Intake m_intake = Intake.getInstance();

  private IntakePivotTarget m_pivotTarget;
  private IntakeState m_intakeState;
  private boolean m_waitForNote;

  public IntakeTask(IntakePivotTarget target, IntakeState state) {
    m_pivotTarget = target;
    m_intakeState = state;
    m_waitForNote = false;
  }

  public IntakeTask(IntakePivotTarget target, IntakeState state, boolean waitForNote) {
    m_pivotTarget = target;
    m_intakeState = state;
    m_waitForNote = waitForNote;
  }

  @Override
  public void start() {
    DriverStation.reportWarning("Auto Intake start", false);
    m_intake.setPivotTarget(m_pivotTarget);
    m_intake.setIntakeState(m_intakeState);
  }

  @Override
  public void update() {
    log(true);

    m_intake.periodic();
  }

  @Override
  public void done() {
    log(false);

    DriverStation.reportWarning("Auto Intake done", false);
  }

  @Override
  public boolean isFinished() {
    boolean waitForNoteDone = !m_waitForNote || m_intake.isHoldingNote();
    return (m_intake.isAtPivotTarget() && waitForNoteDone) || !RobotBase.isReal();
  }
}
