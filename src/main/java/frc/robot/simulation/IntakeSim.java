package frc.robot.simulation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class IntakeSim {
  private static IntakeSim m_intakeSim = null;

  private final DCMotor k_pivotMotor = DCMotor.getNEO(1);
  private final double k_pivotGearRatio = 125.0;

  private final double k_simOffset = -186.525574;

  @SuppressWarnings("unused")
  private final SingleJointedArmSim m_joint = new SingleJointedArmSim(
      k_pivotMotor,
      k_pivotGearRatio,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(Constants.Intake.k_length),
          Constants.Intake.k_mass),
      Constants.Intake.k_length,
      Constants.Intake.k_minAngle,
      Constants.Intake.k_maxAngle,
      true,
      Constants.Intake.k_startingAngle);

  private Mechanism2d m_mech2d = null;

  private final Translation2d m_origin = new Translation2d(
      (Constants.Simulation.k_width / 2) - Constants.Intake.k_distanceFromCenter, 0);

  private MechanismRoot2d m_intakeBase = null;

  private MechanismLigament2d m_intakePivot = null;

  private MechanismLigament2d m_intake = null;

  public static IntakeSim getInstance(Mechanism2d mech2d) {
    if (m_intakeSim == null) {
      m_intakeSim = new IntakeSim(mech2d);
    }
    return m_intakeSim;
  }

  private IntakeSim(Mechanism2d mech2d) {
    m_mech2d = mech2d;
    m_intakeBase = m_mech2d.getRoot("IntakePivot", m_origin.getX(), Constants.Robot.k_bumperStart);
    m_intakePivot = m_intakeBase.append(
        new MechanismLigament2d(
            "IntakePivot",
            Constants.Intake.k_pivotHeight,
            90,
            4,
            new Color8Bit(Color.kRed)));
    m_intake = m_intakePivot.append(
        new MechanismLigament2d(
            "Intake",
            Constants.Intake.k_length,
            k_simOffset,
            4,
            new Color8Bit(Color.kBlue)));
  }

  public void updateAngle(double intakeAngle) {
    m_intake.setAngle(-(k_simOffset + intakeAngle));

    // Translation2d setpoint = m_origin.plus(new Translation2d(x, y));
    // m_crosshair.setPosition(setpoint.getX(), setpoint.getY());

    SmartDashboard.putNumber("Sim/Intake Sim Angle", intakeAngle);
  }
}
