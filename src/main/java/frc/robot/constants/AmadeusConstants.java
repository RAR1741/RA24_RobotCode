package frc.robot.constants;

public final class AmadeusConstants extends Constants {
  public AmadeusConstants() {
    Auto.k_maxModuleSpeed = 5.0;
    Auto.Timing.k_intakeBounceTime = 0.2;

    Vision.Rotation.k_P = 7;

    SwerveDrive.k_maxBoostSpeed = 5.0;
    SwerveDrive.k_slowScaler = 0.2;
    SwerveDrive.k_wheelRadiusIn = 2.0;
    SwerveDrive.k_driveGearRatio = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

    SwerveDrive.Drive.k_P = 0.00079049;
    SwerveDrive.Drive.k_FFS = 0.21465;
    SwerveDrive.Drive.k_FFV = 2.3914;
    SwerveDrive.Drive.k_FFA = 0.36664;

    SwerveDrive.Turn.k_FLOffset = 0.159;
    SwerveDrive.Turn.k_FROffset = 0.163;
    SwerveDrive.Turn.k_BROffset = 0.832;
    SwerveDrive.Turn.k_BLOffset = 0.157;

    SwerveDrive.Turn.k_P = 5.6906;
    SwerveDrive.Turn.k_D = 0.1976;
    SwerveDrive.Turn.k_FFS = 0.29745;
    SwerveDrive.Turn.k_FFV = 0.43892;
    SwerveDrive.Turn.k_FFA = 0.048573;

    Shooter.k_maxRPM = 5000.0;
    Shooter.k_passRPM = 4000.0;

    Shooter.ShootPID.k_shooterMotorP = 0.0009300;
    Shooter.ShootPID.k_shooterMotorI = 0.00000008;
    Shooter.ShootPID.k_shooterMotorD = 0.0001000;
    Shooter.ShootPID.k_shooterMotorFF = 0.00015;

    Shooter.k_shooterMinOutput = 0.0;
    Shooter.k_minAngle = 20;
    Shooter.k_absPivotOffset = 208.587 - 90;
    Shooter.k_initialPivotOffset = -1.75;
  }
}
