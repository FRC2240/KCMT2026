package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCommands {
  public static Supplier<ChassisSpeeds> controllerDriveSupplier(CommandXboxController controller) {
    return () -> {
      return new ChassisSpeeds(
        DriveConstants.MAX_SPEED.times(controller.getLeftX()), 
        DriveConstants.MAX_SPEED.times(controller.getLeftY()), 
        RadiansPerSecond.of(0)
      );
    };
  }

  public static Supplier<ChassisSpeeds> controllerTurnSupplier(CommandXboxController controller) {
    return () -> {
      return new ChassisSpeeds(
        MetersPerSecond.of(0), 
        MetersPerSecond.of(0), 
        DriveConstants.MAX_ANGULAR_RATE.times(controller.getRightX())
      );
    };
  }

  public static Command controlDrivetrainWithController(Drivetrain drivetrain, CommandXboxController controller) {
    return drivetrain.driveCommand(List.of(controllerDriveSupplier(controller), controllerTurnSupplier(controller)));
  }
}
