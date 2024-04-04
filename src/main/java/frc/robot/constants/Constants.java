package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static class ClimberConstants {
    public int k_leftMotorID = 19;
    public int k_rightMotorID = 18;

    public double k_raiseSpeed = 600.0; // RPM
    public double k_lowerSpeed = -600.0; // RPM

    public double k_gearRatio = 1.0 / 16.0;

    public double k_P = 0.001;
    public double k_I = 0.0;
    public double k_D = 0.0;

    public double k_minOutput = -0.5;
    public double k_maxOutput = 0.5;
  }

  public static class IntakeConstants {
    public int k_pivotMotorId = 13;
    public int k_intakeMotorId = 14;

    public double k_pivotMotorMaxOutput = 0.2;

    public int k_pivotEncoderId = 4;

    public double k_stowPivotAngle = 320.6;
    public double k_groundPivotAngle = 99.5;
    public double k_sourcePivotAngle = 240.0;
    public double k_ejectPivotAngle = 160.0;
    public double k_ampPivotAngle = 160.0;

    public double k_intakeSpeed = 0.4;
    public double k_ejectSpeed = -0.525;
    public double k_feedShooterSpeed = -1.0;

    public double k_pivotMotorP = 0.035;
    public double k_pivotMotorI = 0.0;
    public double k_pivotMotorD = 0.0;
    public double k_pivotMotorIZone = 0.0;

    public double k_pivotMotorKS = 0.0; // volts
    public double k_pivotMotorKG = 0.03; // volts
    public double k_pivotMotorKV = 0.244; // volts*sec / rad
    public double k_pivotMotorKA = 0.0; // 0.02; // volts*sec^2 / rad

    public double k_length = 14.319626; // In inches
    public double k_mass = 5.8967; // In kg

    public double k_pivotHeight = 5.75;

    public double k_minAngle = 0.0;
    public double k_maxAngle = 0.0;
    public double k_startingAngle = 0.0;

    public double k_distanceFromCenter = 12.5;
    public int k_sensorThreshold = 1000; // 1,000 is "in", 2000 is max

    public double k_pivotOffset = 127.5;

    public double k_maxVelocity = 690.0;
    public double k_maxAcceleration = 1380.0;
  }

  public static class SimulationConstants {
    public double k_width = 150.0; // Inches
    public double k_height = 100.0; // Inches
  }

  public static class FieldConstants {
    public double k_width = Units.feetToMeters(54.0);
    public double k_length = Units.feetToMeters(27.0);

    public double k_ampBottom = 26;
    public double k_ampTop = 44;

    public double k_speakerBottom = 78;
    public double k_speakerTop = 82.875;
    public double k_speakerAngle = 14.0;

    // TODO: Maybe get these from AprilTags?
    private double speakerHeight = 2.032; // Meters
    public Pose3d k_redSpeakerPose = new Pose3d(16.579342, 5.547868, speakerHeight, new Rotation3d());
    public Pose3d k_blueSpeakerPose = new Pose3d(-0.0381, 5.547868, speakerHeight, new Rotation3d());

    public Pose2d k_redPassPose = new Pose2d(14.71, 6.0, new Rotation2d());
    public Pose2d k_bluePassPose = new Pose2d(1.75, 6.0, new Rotation2d());
    public double k_passAngle = 22.5;
  }
}
