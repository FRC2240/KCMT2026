package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public class DriveConstants {
  public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts.times(0.9);
  public static final AngularVelocity MAX_ANGULAR_RATE = RotationsPerSecond.of(1);

  public static final LinearVelocity MAX_SLOW_SPEED = MAX_SPEED.div(5);
  public static final AngularVelocity MAX_SLOW_ANGULAR_RATE = MAX_ANGULAR_RATE.div(5);

  public static final PIDController TRANSLATION_PID_CONTROLLER = new PIDController(5, 0, 1);
  public static final PIDController ROTATION_PID_CONTROLLER = new PIDController(6,0,.5);

}
