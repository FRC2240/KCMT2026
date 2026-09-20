package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
  @AutoLog
  public class ModuleIOInputs {
    public SwerveModuleState state;
    public SwerveModulePosition position;
    public SwerveModulePosition[] positionSamples; 
    public double[] timestamps;
  }

  public default void updateInputs(ModuleIOInputs inputs) {};

  public default void setState(SwerveModuleState state) {};

  public default Rotation2d getHeading() { return null; };
}
