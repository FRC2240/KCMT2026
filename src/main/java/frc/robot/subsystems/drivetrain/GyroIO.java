package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
  @AutoLog
  public class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yaw = Rotation2d.kZero;
    public Rotation2d[] samples;
    public double[] timestamps;
  }

  public default void updateInputs(GyroIOInputs inputs) {};
}
