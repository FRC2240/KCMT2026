package frc.robot.subsystems.bludsystem;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Bludsystem extends SubsystemBase {
  private TalonFX motor = new TalonFX(5);
  private PositionTorqueCurrentFOC positionReq = new PositionTorqueCurrentFOC(0);
  private StatusSignal<Angle> position = motor.getPosition();

  public Bludsystem() {
    //TalonFXConfiguration conf = new TalonFXConfiguration();

  }



  public Command untagle() {
    return run(() -> {

    });
  }

  public void setPostion(Angle position) {
    motor.setControl(positionReq.withPosition(position.times(312.5)));
  }

  public Command moveToPosition(Supplier<Angle> rotationSupplier) {
    return Commands.repeatingSequence(
      run(() -> {
        setPostion(rotationSupplier.get());
      }).until(() -> (position.getValue().lt(Rotations.of(-5)) || position.getValue().gt(Rotations.of(5)))),
      run(() -> {
        setPostion(Rotations.of(0));
      }).until(() -> position.refresh().getValue().isNear(Rotations.of(0), Rotations.of(0.05)))
    );
  }
}
