package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public class IntakePivotConstants {
  public static final int PIVOT_MOTOR_ID = 55;
  public static final int PIVOT_TOLLERANCE = 1;

  public static final Angle PIVOT_EXTENDED_POSITION = Rotations.of(-10.495); 
  public static final Angle PIVOT_RAMP_POSITION = Rotations.of(-1.0);
  public static final Angle PIVOT_RAMP_BOTTOM_POSITION = Rotations.of(-8.5);

  public static Current SUPPLY_CURRENT_LIMIT = Amps.of(45);
}
