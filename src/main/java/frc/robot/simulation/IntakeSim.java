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

  private final Translation2d m_origin = new Translation2d((Constants.Simulation.k_width / 2) - Constants.Intake.k_distanceFromCenter, 0);

  private final MechanismRoot2d m_intakeBase = m_mech2d.getRoot("IntakePivot", m_origin.getX(),
      Constants.Intake.k_pivotHeight);

  private final MechanismRoot2d m_crosshair = m_mech2d.getRoot("Crosshair", m_origin.getX(), m_origin.getY());

  private final MechanismLigament2d m_intakePivot = m_intakeBase.append(
      new MechanismLigament2d(
          "IntakePivot",
          Constants.Intake.k_pivotHeight,
          90,
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
    m_intake.setAngle(k_simOffset + intakeAngle);

    // Translation2d setpoint = m_origin.plus(new Translation2d(x, y));
    // m_crosshair.setPosition(setpoint.getX(), setpoint.getY());

    double x = Math.cos(Units.degreesToRadians(intakeAngle - k_simOffset + 90)) * Constants.Intake.k_length;
    double y = Math.sin(Units.degreesToRadians(intakeAngle - k_simOffset + 90)) * Constants.Intake.k_length;

    Translation2d setpoint = m_origin.plus(new Translation2d(x, y + Constants.Intake.k_pivotHeight)); // Good enough, I guess (the crosshair drifts a little)
    m_crosshair.setPosition(setpoint.getX(), setpoint.getY());
    
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

    // Draw crosshair
    final double k_crosshairLength = 2;
    final double k_crosshairThickness = 1;
    final Color8Bit k_crosshairColor = new Color8Bit(Color.kOrange);

    final String[] k_directions = { "Right", "Top", "Left", "Bottom" };
    double crosshairAngle = 0;
    for (String direction : k_directions) {
      m_crosshair.append(
        new MechanismLigament2d(
          "Crosshair" + direction,
          k_crosshairLength,
          crosshairAngle,
          k_crosshairThickness,
          k_crosshairColor
        ));
      crosshairAngle += 90;
    }
  }
}