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

  private final double k_simOffset = 0.0;

  private final SingleJointedArmSim m_joint = new SingleJointedArmSim(
      k_pivotMotor,
      k_pivotGearRatio,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(Constants.Intake.k_length), Constants.Intake.k_mass),
      Units.inchesToMeters(Constants.Intake.k_length),
      Constants.Intake.k_minAngle,
      Constants.Intake.k_maxAngle,
      true,
      Constants.Intake.k_startingAngle);

  private final Mechanism2d m_mech2d = new Mechanism2d(Constants.Simulation.k_width, Constants.Simulation.k_height);

  private final Translation2d m_origin = new Translation2d(Constants.Simulation.k_width / 2.0, 0);

  private final MechanismRoot2d m_intakePivot = m_mech2d.getRoot("IntakePivot", m_origin.getX(),
      Constants.Intake.k_pivotHeight);

  private final MechanismLigament2d m_intakeBase = m_intakePivot.append(
      new MechanismLigament2d(
          "IntakeBase",
          Constants.Intake.k_pivotHeight,
          -90,
          4,
          new Color8Bit(Color.kBlue)));

  private final MechanismLigament2d m_intake = m_intakePivot.append(
      new MechanismLigament2d(
          "Intake",
          Constants.Intake.k_length,
          k_simOffset,
          4,
          new Color8Bit(Color.kYellow)));

  public static IntakeSim getInstance() {
    if (m_intakeSim == null) {
      m_intakeSim = new IntakeSim();
    }
    return m_intakeSim;
  }

  private IntakeSim() {
    addAdditionalDrawings();

    SmartDashboard.putData("Intake Sim", m_mech2d);
  }

  public void updateIntakePosition(double intakeAngle) {
    m_intake.setAngle(k_simOffset + intakeAngle); //TODO The intake will likely handle angles oddly, so this will 100% need modified
    
    SmartDashboard.putNumber("Intake Sim Angle", intakeAngle);
  }

  private void addAdditionalDrawings() {
    // Draw the robot's bumpers
    double bumperPosition = Constants.Simulation.k_width / 2 - Constants.Robot.k_length / 2;
    m_mech2d.getRoot("Robot", bumperPosition, Constants.Robot.k_bumperStart).append(
        new MechanismLigament2d(
            "RobotBase",
            Constants.Robot.k_length,
            0,
            Constants.Robot.k_bumperHeight,
            new Color8Bit(Color.kRed)));
  }
}