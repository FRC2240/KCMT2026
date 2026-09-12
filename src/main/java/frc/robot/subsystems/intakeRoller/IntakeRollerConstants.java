package frc.robot.subsystems.intakeRoller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class IntakeRollerConstants {
  public static final int INTAKE_MOTOR_ID = 54; 
  public static final int INTAKE_MOTOR_FOLLOWER_ID = 59;

  public static final int PIVOT_MOTOR_ID = 55;
  public static final int PIVOT_TOLLERANCE = 1;

  public static final Current STATOR_CURRENT_LIMIT = Amps.of(85);
  public static final AngularVelocity INTAKE_VELOCITY = RotationsPerSecond.of(-60); 
}