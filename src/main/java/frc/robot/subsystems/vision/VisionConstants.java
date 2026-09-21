package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
  public static Distance MAX_HEIGHT_ERROR = Meters.of(0.2);

  // Standard deviation coefficents, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double LINEAR_STDDEV_COEFF = 0.02; // Meters
  public static double ANGULAR_STDDEV_COEFF = 0.06; // Radians
}
