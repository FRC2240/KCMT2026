package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Radians;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(4, CANBus.roboRIO());
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> samples;
  private final Queue<Double> timestamps;

  public GyroIOPigeon2() {
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(250.0);
    pigeon.optimizeBusUtilization();

    samples = OdometryThread.getInstance().registerSignal(yaw.clone());
    timestamps = OdometryThread.getInstance().createTimestampQueue();
  }

  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw).equals(StatusCode.OK);
    inputs.yaw = pigeon.getRotation2d();
    inputs.samples = this.samples.stream().map((Double value) -> Radians.of(value)).toArray(Rotation2d[]::new);
    inputs.timestamps = this.timestamps.stream().mapToDouble((Double value) -> value).toArray();

    this.samples.clear();
    this.timestamps.clear();
  }
}
