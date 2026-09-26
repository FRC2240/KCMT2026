package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveSuppliers {
  private Drivetrain drivetrain;
  private CommandXboxController controller;
  private boolean slowModeEnabled = false;

  public DriveSuppliers(Drivetrain drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
  }

  private int getAllianceMultiplier() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) ? -1 : 1;
  }

  private double threshold(double value, double threshold) {
    if (Math.abs(value) < threshold) return 0;
    return value;
  }

  private double delinearize(double value, double exponent) {
    return Math.pow(Math.abs(value), exponent) * (value < 0 ? -1 : 1);
  }

  
  private LinearVelocity getMaxDriveSpeed() {
    return slowModeEnabled ? DriveConstants.MAX_SLOW_SPEED : DriveConstants.MAX_SPEED;
  }

  private AngularVelocity getMaxRotationSpeed() {
    return slowModeEnabled ? DriveConstants.MAX_SLOW_ANGULAR_RATE : DriveConstants.MAX_ANGULAR_RATE;
  }

  public Supplier<ChassisSpeeds> controllerDrive() {
    return () -> {
      return new ChassisSpeeds(
        DriveConstants.MAX_SPEED.times(delinearize(threshold(controller.getLeftY(), 0.1), 1.5)).times(getAllianceMultiplier()),
        DriveConstants.MAX_SPEED.times(delinearize(threshold(controller.getLeftX(), 0.1), 1.5)).times(getAllianceMultiplier()), 
        RadiansPerSecond.of(0)
      );
    };
  }

  public Supplier<ChassisSpeeds> controllerTurn() {
    return () -> {
      return new ChassisSpeeds(
        MetersPerSecond.of(0), 
        MetersPerSecond.of(0), 
        DriveConstants.MAX_ANGULAR_RATE.times(threshold(controller.getRightX(), 0.1) * -1)
      );
    };
  }

  public Supplier<ChassisSpeeds> driveToPoint(Supplier<Translation2d> targetTranslationSupplier) {
    return () -> {
      Translation2d targetTranslation = targetTranslationSupplier.get();
      Translation2d currentTranslation = drivetrain.getTranslation();
      Translation2d translationToTarget = currentTranslation.minus(targetTranslation);

      Rotation2d directionOfTravel = translationToTarget.getAngle();
      double linearDistance = currentTranslation.minus(targetTranslation).getNorm();

      double velocityOutput = DriveConstants.TRANSLATION_PID_CONTROLLER.calculate(linearDistance, 0);
      // Limit the resulting velocity
      velocityOutput = Math.min(velocityOutput, getMaxDriveSpeed().in(MetersPerSecond));

      double vx = velocityOutput * directionOfTravel.getCos();
      double vy = velocityOutput * directionOfTravel.getSin();

      return new ChassisSpeeds(vx, vy, 0);
    };
  }

  public Supplier<ChassisSpeeds> rotateToRotation(Supplier<Rotation2d> targetRotationSupplier) {
    return () -> {
      Rotation2d targetRotation = targetRotationSupplier.get();
      Rotation2d currentRotation = drivetrain.getRotation();
      double rotationDistance = currentRotation.minus(targetRotation).getRadians();

      double rotationalOutput = DriveConstants.ROTATION_PID_CONTROLLER.calculate(rotationDistance, 0);
      // Clamps the output to the max rotational speed
      rotationalOutput = Math.max(Math.min(rotationalOutput, getMaxRotationSpeed().in(RadiansPerSecond)), -getMaxRotationSpeed().in(RadiansPerSecond));
      return new ChassisSpeeds(0, 0, rotationalOutput);
    };
  }

  public Supplier<ChassisSpeeds> rotateToFacePoint(Supplier<Translation2d> pointSupplier) {
    return rotateToRotation(() -> {
      Translation2d targetTranslation = pointSupplier.get();
      Translation2d currentTranslation = drivetrain.getTranslation();
      Translation2d translationToTarget = targetTranslation.minus(currentTranslation);
      Rotation2d rotationToPoint = translationToTarget.getAngle();
      return rotationToPoint;
    });
  }

  public Supplier<ChassisSpeeds> driveToDistanceFromPoint(Supplier<Translation2d> pointSupplier, Supplier<Distance> distanceSupplier) {
    return driveToPoint(() -> {
      Translation2d point = pointSupplier.get();
      Distance distance = distanceSupplier.get();

      Translation2d currentTranslation = drivetrain.getTranslation();
      Rotation2d angle = currentTranslation.minus(point).getAngle();

      return new Translation2d(distance.in(Meters), angle).plus(point);
    });
  }

  public Supplier<ChassisSpeeds> trenchAlign() {
    return rotateToRotation(() -> {
      double currentRotation = Math.abs(drivetrain.getRotation().getDegrees());

      if (currentRotation >= 90) {
        return Rotation2d.fromDegrees(180);
      }
      return Rotation2d.fromDegrees(0);
    });
  }
  
}
