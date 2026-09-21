package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface CameraIO {
  @AutoLog
  public class CameraIOInputs {
    public boolean connected = false;
    public Estimate[] estimates;
  }

  public record Estimate (
    Pose3d pose,
    double timestamp,
    int tagCount,
    double averageTagDistance
  ) {};

  public default void updateInputs(CameraIOInputs inputs) {};
}
