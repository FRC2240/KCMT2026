package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCommands {

  public Drivetrain drivetrain;

  private boolean slowModeEnabled = false;

  private static int getAllianceMultiplier() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) ? -1 : 1;
  }

  private static double threshold(double value, double threshold) {
    if (Math.abs(value) < threshold) return 0;
    return value;
  }

  /**
     * Gets the maximum drive speed, accounting for if the robot is in slow mode.
     * 
     * @return The current maximum speed
     */
    public LinearVelocity getMaxDriveSpeed() {
        return slowModeEnabled ? DriveConstants.MAX_SLOW_SPEED : DriveConstants.MAX_SPEED;
    }


  public static Supplier<ChassisSpeeds> controllerDriveSupplier(CommandXboxController controller) {
    return () -> {
      return new ChassisSpeeds(
        DriveConstants.MAX_SPEED.times(threshold(controller.getLeftY(), 0.1)).times(getAllianceMultiplier()),
        DriveConstants.MAX_SPEED.times(threshold(controller.getLeftX(), 0.1)).times(getAllianceMultiplier()), 
        RadiansPerSecond.of(0)
      );
    };
  }

  public static Supplier<ChassisSpeeds> controllerTurnSupplier(CommandXboxController controller) {
    return () -> {
      return new ChassisSpeeds(
        MetersPerSecond.of(0), 
        MetersPerSecond.of(0), 
        DriveConstants.MAX_ANGULAR_RATE.times(threshold(controller.getRightX(), 0.1) * -1)
      );
    };
  }

  public static Command controlDrivetrainWithController(Drivetrain drivetrain, CommandXboxController controller) {
    return drivetrain.driveCommand(List.of(controllerDriveSupplier(controller), controllerTurnSupplier(controller)));
  }

  public Supplier<ChassisSpeeds> driveToPoint(Supplier<Translation2d> targetTranslationSupplier) {
        return () -> {
            Translation2d targetTranslation = targetTranslationSupplier.get();
            Translation2d currentTranslation = drivetrain.getTranslation();
            Translation2d translationToTarget = targetTranslation.minus(currentTranslation);

            Rotation2d directionOfTravel = translationToTarget.getAngle();
            double linearDistance = targetTranslation.minus(currentTranslation).getNorm();

            double velocityOutput = DriveConstants.TRANSLATION_PID_CONTROLLER.calculate(linearDistance, 0);
            // Limit the resulting velocity
            velocityOutput = Math.min(velocityOutput, getMaxDriveSpeed().in(MetersPerSecond));

            double vx = velocityOutput * directionOfTravel.getCos();
            double vy = velocityOutput * directionOfTravel.getSin();

            return new ChassisSpeeds(vx, vy, 0);
        };
    }
}
