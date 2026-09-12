package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class IntakePivotIOReal implements IntakePivotIO {
  private final TalonFX motor = new TalonFX(IntakePivotConstants.PIVOT_MOTOR_ID);
  private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0);
  private final MotionMagicTorqueCurrentFOC mmRequest = new MotionMagicTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

  private final StatusSignal<AngularVelocity> velocitySignal = motor.getVelocity();
  private final StatusSignal<Angle> positionSignal = motor.getPosition();
  private final StatusSignal<Current> supplyCurrentSignal = motor.getSupplyCurrent();
  private final StatusSignal<Current> statorCurrentSignal = motor.getStatorCurrent();

  public IntakePivotIOReal() {
    TalonFXConfiguration conf = new TalonFXConfiguration();
    conf.MotionMagic.MotionMagicCruiseVelocity = 10;
    conf.MotionMagic.MotionMagicAcceleration = 16;

    // For extension
    conf.Slot0.kP = 25;
    conf.Slot0.kI = 7;
    conf.Slot0.kD = 10;

    // Slot 2 has a small PID for rezeroing
    conf.Slot2.kP = 5;

    conf.CurrentLimits.SupplyCurrentLimit = IntakePivotConstants.SUPPLY_CURRENT_LIMIT.in(Amps);

    motor.getConfigurator().apply(conf);
    motor.setPosition(Rotations.of(0));
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocitySignal, positionSignal, supplyCurrentSignal, statorCurrentSignal);

    inputs.velocity = velocitySignal.getValue();
    inputs.position = positionSignal.getValue();
    inputs.supplyCurrent = supplyCurrentSignal.getValue();
    inputs.statorCurrent = statorCurrentSignal.getValue();
  }

  @Override
  public void resetPositionMeasurement(Angle position) {
    motor.setPosition(position);
  }

  @Override
  public void setPosition(Angle position) {
    motor.setControl(positionRequest.withPosition(position));
  }

  @Override
  public void setPositionMotionMagic(Angle position) {
    motor.setControl(mmRequest.withPosition(position));
  }

  @Override
  public void setVelocityWithSlot(AngularVelocity velocity, int slot) {
    motor.setControl(velocityRequest.withVelocity(velocity).withSlot(slot));
  }

}
