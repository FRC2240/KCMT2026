package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCommands {
  private static int getAllianceMultiplier() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) ? -1 : 1;
  }

  private static double threshold(double value, double threshold) {
    if (Math.abs(value) < threshold) return 0;
    return value;
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
}
