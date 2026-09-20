package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder encoder;

  private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC turnPositionRequest = new PositionTorqueCurrentFOC(0);

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Angle> turnPosition;

  private final Queue<Double> driveSamples;
  private final Queue<Double> turnSamples;
  private final Queue<Double> timestamps;

  private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

  public ModuleIOTalonFX(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
    this.constants = constants;

    driveMotor = new TalonFX(constants.DriveMotorId, TunerConstants.kCANBus);
    turnMotor = new TalonFX(constants.SteerMotorId, TunerConstants.kCANBus);
    encoder = new CANcoder(constants.EncoderId, TunerConstants.kCANBus);

    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    turnPosition = turnMotor.getPosition();

    StatusSignal.setUpdateFrequencyForAll(250.0, drivePosition, turnPosition, driveVelocity);

    TalonFXConfiguration driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveMotor.getConfigurator().apply(driveConfig);
    driveMotor.setPosition(0.0);

    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = constants.SteerMotorGains;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted = 
        constants.SteerMotorInverted 
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnMotor.getConfigurator().apply(turnConfig);

    CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    encoder.getConfigurator().apply(cancoderConfig);

    driveSamples = OdometryThread.getInstance().registerSignal(drivePosition.clone());
    turnSamples = OdometryThread.getInstance().registerSignal(turnPosition.clone());
    timestamps = OdometryThread.getInstance().createTimestampQueue();
  }

  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(drivePosition, turnPosition);

    inputs.position = new SwerveModulePosition(
      drivePosition.getValueAsDouble() * constants.WheelRadius,
      Rotation2d.fromRotations(turnPosition.getValueAsDouble())
    );

    inputs.state = new SwerveModuleState(MetersPerSecond.of(driveVelocity.getValueAsDouble() * constants.WheelRadius), Rotation2d.fromRotations(turnPosition.getValueAsDouble()));
    

    if (driveSamples.size() == turnSamples.size() && turnSamples.size() == timestamps.size()) {
      int n = driveSamples.size();
      inputs.positionSamples = new SwerveModulePosition[n];
      inputs.timestamps = new double[n];
      for (int i = 0; i < n; i++) {
        inputs.positionSamples[i] = new SwerveModulePosition(
          driveSamples.remove() * constants.WheelRadius,
          Rotation2d.fromRotations(turnSamples.remove())
        );
        inputs.timestamps[i] = timestamps.remove();
      }
    } else {
      inputs.positionSamples = new SwerveModulePosition[0];
      inputs.timestamps = new double[0];
    }

    driveSamples.clear();
    turnSamples.clear();
    timestamps.clear();
  }

  public void setState(SwerveModuleState state) {
    driveMotor.setControl(driveVelocityRequest.withVelocity(state.speedMetersPerSecond / constants.WheelRadius));
    turnMotor.setControl(turnPositionRequest.withPosition(state.angle.getRotations()));
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromRotations(turnPosition.getValueAsDouble());
  }
}
