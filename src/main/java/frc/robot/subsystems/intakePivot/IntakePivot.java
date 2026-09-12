package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake Pivot", inputs);
  }

  public void extend() {
    io.setPosition(IntakePivotConstants.PIVOT_EXTENDED_POSITION);
  }

  public void ramp() {
    io.setPosition(IntakePivotConstants.PIVOT_RAMP_POSITION);
  }

  public Command extendCommand() {
    return runOnce(this::extend);
  }

  public Command contractCommand() {
    return runOnce(() -> io.setPositionMotionMagic(Rotations.of(0))).andThen(run(() -> {}));
  }

  public Command rezeroCommand() {
    return Commands.sequence(
        runOnce(() -> io.setVelocityWithSlot(RotationsPerSecond.of(-2), 2)),
        Commands.waitSeconds(0.3),
        Commands.waitUntil(() -> Math.abs(inputs.velocity.in(RotationsPerSecond)) < 0.2),
        runOnce(() -> io.resetPositionMeasurement(IntakePivotConstants.PIVOT_EXTENDED_POSITION)),
        extendCommand());
  }

  public Command rampCommand() {
    return Commands.repeatingSequence(
      runOnce(this::ramp),
      Commands.waitSeconds(1.1),
      runOnce(this::extend),
      Commands.waitSeconds(1.1)
    );
  }
}
