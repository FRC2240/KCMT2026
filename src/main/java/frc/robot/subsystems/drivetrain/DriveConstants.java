package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public class DriveConstants {
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
        Math.max(
          Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
          Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
        Math.max(
          Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
          Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  public static Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };

  public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
  public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(MAX_SPEED.in(MetersPerSecond) / DRIVE_BASE_RADIUS);

  public static int SWERVE_UPDATE_FREQUENCY = 250;

  public static final PIDController TRANSLATION_PID_CONTROLLER = new PIDController(5, 0, 1);
  public static final PIDController ROTATION_PID_CONTROLLER = new PIDController(6,0,.5);


  public static final double ROBOT_MASS_KG = 74.088;
  public static final double ROBOT_MOI = 6.883;
  public static final double WHEEL_COF = 1.2;
  public static final RobotConfig PP_CONFIG =
    new RobotConfig(
      ROBOT_MASS_KG,
      ROBOT_MOI,
      new ModuleConfig(
        TunerConstants.FrontLeft.WheelRadius,
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
        WHEEL_COF,
        DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
        TunerConstants.FrontLeft.SlipCurrent,
        1),
      MODULE_TRANSLATIONS);
}
