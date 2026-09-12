package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

// Non-physics based sim. To be updated

public class IntakePivotIOSim implements IntakePivotIO {
  Angle position = Rotations.of(0);
  AngularVelocity velocity = RotationsPerSecond.of(0);

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.position = position;
    inputs.velocity = velocity;
    inputs.statorCurrent = Amps.of(0);
    inputs.supplyCurrent = Amps.of(0);
  }

  @Override
  public void resetPositionMeasurement(Angle position) {
    this.position = position;
    velocity = RotationsPerSecond.of(0);
  }

  @Override
  public void setPosition(Angle position) {
    this.position = position;
    velocity = RotationsPerSecond.of(0);
  }

  @Override
  public void setPositionMotionMagic(Angle position) {
    this.position = position;
    velocity = RotationsPerSecond.of(0);
  }

  @Override
  public void setVelocityWithSlot(AngularVelocity velocity, int slot) {
    this.velocity = velocity;
  }


}
