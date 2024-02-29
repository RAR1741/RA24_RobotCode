package frc.robot.subsystems.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class CustomHolonomicDriveController2 extends PPHolonomicDriveController {
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController rotationController;
  private final double maxModuleSpeed;
  private final double mpsToRps;

  private boolean isEnabled = true;

  private static Supplier<Optional<Rotation2d>> rotationTargetOverride = null;

  public CustomHolonomicDriveController2(
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      double period,
      double maxModuleSpeed,
      double driveBaseRadius) {
    super(translationConstants, rotationConstants, period, maxModuleSpeed, driveBaseRadius);

    this.xController = new PIDController(
        translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
    // this.xController.setIntegratorRange(-translationConstants.iZone,
    // translationConstants.iZone);

    this.yController = new PIDController(
        translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
    // this.yController.setIntegratorRange(-translationConstants.iZone,
    // translationConstants.iZone);

    // Temp rate limit of 0, will be changed in calculate
    // this.rotationController = new PIDController(
    // rotationConstants.kP,
    // rotationConstants.kI,
    // rotationConstants.kD,
    // period);
    this.rotationController = new ProfiledPIDController(
        rotationConstants.kP,
        rotationConstants.kI,
        rotationConstants.kD,
        new TrapezoidProfile.Constraints(0, 0),
        period);
    this.rotationController.setIntegratorRange(-rotationConstants.iZone, rotationConstants.iZone);
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

    this.maxModuleSpeed = maxModuleSpeed;
    this.mpsToRps = 1.0 / driveBaseRadius;
  }

  public CustomHolonomicDriveController2(
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      double maxModuleSpeed,
      double driveBaseRadius) {
    this(translationConstants, rotationConstants, 0.02, maxModuleSpeed, driveBaseRadius);
  }

  private double copyRotationFF = 0.0;
  private double copyRotationFeedback = 0.0;
  private double copyRotationTarget = 0.0;
  private double copyRotationActual = 0.0;
  boolean firstTimeForEverything = true;

  @Override
  public ChassisSpeeds calculateRobotRelativeSpeeds(
      Pose2d currentPose, PathPlannerTrajectory.State targetState) {
    if (firstTimeForEverything) {
      firstTimeForEverything = false;
      rotationController.reset(currentPose.getRotation().getRadians());
    }
    double xFF = targetState.velocityMps * targetState.heading.getCos();
    double yFF = targetState.velocityMps * targetState.heading.getSin();

    if (!this.isEnabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, 0, currentPose.getRotation());
    }

    double xFeedback = this.xController.calculate(currentPose.getX(), targetState.positionMeters.getX());
    double yFeedback = this.yController.calculate(currentPose.getY(), targetState.positionMeters.getY());

    double angVelConstraint = targetState.constraints.getMaxAngularVelocityRps();
    double maxAngVel = angVelConstraint;

    if (Double.isFinite(maxAngVel)) {
      // Approximation of available module speed to do rotation with
      double maxAngVelModule = Math.max(0, maxModuleSpeed - targetState.velocityMps) * mpsToRps;
      maxAngVel = Math.min(angVelConstraint, maxAngVelModule);
    }

    var rotationConstraints = new TrapezoidProfile.Constraints(
        maxAngVel, targetState.constraints.getMaxAngularAccelerationRpsSq());

    Rotation2d targetRotation = targetState.targetHolonomicRotation;
    if (rotationTargetOverride != null) {
      targetRotation = rotationTargetOverride.get().orElse(targetRotation);
    }

    double rotationFeedback = 0.0;
    // for (int i = 0; i < 5; i++) {
    rotationFeedback = rotationController.calculate(
        currentPose.getRotation().getRadians(),
        new TrapezoidProfile.State(targetRotation.getRadians(), 0),
        rotationConstraints);
    // }

    copyRotationTarget = targetRotation.getDegrees();
    copyRotationActual = currentPose.getRotation().getDegrees();

    copyRotationFeedback = rotationFeedback;
    double rotationFF = targetState.holonomicAngularVelocityRps.orElse(rotationController.getSetpoint().velocity);
    copyRotationFF = rotationFF;

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, currentPose.getRotation());
  }

  public double getRotationFF() {
    return copyRotationFF;
  }

  public double getRotationFeedback() {
    return copyRotationFeedback;
  }

  public double getRotationTarget() {
    return copyRotationTarget;
  }

  public double getRotationActual() {
    return copyRotationActual;
  }
}
