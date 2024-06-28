package frc.robot.constants;

public final class ApolloConstants extends Constants {
  public ApolloConstants() {
    Auto.k_maxModuleSpeed = 4.6;
    Auto.Timing.k_intakeBounceTime = 0.3;

    Vision.Rotation.k_P = 4.0;

    SwerveDrive.k_maxBoostSpeed = 4.5;
    SwerveDrive.k_slowScaler = 0.3;

    SwerveDrive.k_wheelRadiusIn = 3.815 / 2.0; // Updated for Vex Pro Wheels
    SwerveDrive.k_driveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

    SwerveDrive.Drive.k_P = 0.000221;
    SwerveDrive.Drive.k_FFS = 0.255;
    SwerveDrive.Drive.k_FFV = 2.675;
    SwerveDrive.Drive.k_FFA = 0.525;

    SwerveDrive.Turn.k_FLOffset = 0.470;
    SwerveDrive.Turn.k_FROffset = 0.753;
    SwerveDrive.Turn.k_BROffset = 0.133;
    SwerveDrive.Turn.k_BLOffset = 0.549;
    SwerveDrive.Turn.k_P = 0.6906;
    SwerveDrive.Turn.k_D = 0.0;

    Shooter.k_maxRPM = 5000.0;
    Shooter.k_passRPM = 3500.0;

    Shooter.ShootPID.k_shooterMotorP = 0.0007000;
    Shooter.ShootPID.k_shooterMotorI = 0.00000008;
    Shooter.ShootPID.k_shooterMotorD = 0.0000500;
    Shooter.ShootPID.k_shooterMotorFF = 0.000150;

    Shooter.AmpPID.k_shooterMotorP = 0.0004300;
    Shooter.AmpPID.k_shooterMotorI = 0.00000008;
    Shooter.AmpPID.k_shooterMotorD = 0.0001000;
    Shooter.AmpPID.k_shooterMotorFF = 0.00010;

    Shooter.k_shooterMinOutput = -1.0;
    Shooter.k_minAngle = 21.0;
    Shooter.k_absPivotOffset = 149.254 - 90;
    Shooter.k_initialPivotOffset = 0.0;

    Shooter.k_pivotMotorP = 2.0;
    Shooter.k_pivotMotorI = 0.0;
    Shooter.k_pivotMotorD = 0.001;
    Shooter.k_pivotMotorIZone = 0.0;
  }
}
