package frc.robot.subsystems.intakePivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public interface IntakePivotIO {
  @AutoLog
  class IntakePivotIOInputs {
    public AngularVelocity velocity;
    public Angle position;
    public Current supplyCurrent;
    public Current statorCurrent;
  }

  public default void updateInputs(IntakePivotIOInputs inputs) {}

  public default void resetPositionMeasurement(Angle position) {}

  public default void setPosition(Angle position) {}
  public default void setPositionMotionMagic(Angle position) {}
  public default void setVelocityWithSlot(AngularVelocity velocity, int slot) {}
}