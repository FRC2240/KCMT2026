package frc.robot.subsystems.drivetrain;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import frc.robot.generated.TunerConstants;

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder encoder;

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<Angle> turnPosition;

  private final Queue<Double> driveSamples;
  private final Queue<Double> turnSamples;
  private final Queue<Double> timestamps;


  public ModuleIOTalonFX(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
    driveMotor = new TalonFX(constants.DriveMotorId, TunerConstants.kCANBus);
    drivePosition = driveMotor.getPosition();

    turnMotor = new TalonFX(constants.SteerMotorId, TunerConstants.kCANBus);
    encoder = new CANcoder(constants.EncoderId, TunerConstants.kCANBus);
    turnPosition = turnMotor.getPosition();

    driveSamples = OdometryThread.getInstance().registerSignal(drivePosition.clone());
    turnSamples = OdometryThread.getInstance().registerSignal(turnPosition.clone());
    timestamps = OdometryThread.getInstance().createTimestampQueue();
  }

  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(drivePosition, turnPosition);

    inputs.position = new SwerveModulePosition();

    driveSamples.clear();
    turnSamples.clear();
    timestamps.clear();
  }

  public void setState(SwerveModuleState state) {

  }

  public Rotation2d getHeading() {
    return null;
  }
}
