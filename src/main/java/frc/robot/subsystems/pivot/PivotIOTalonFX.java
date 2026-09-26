package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class PivotIOTalonFX implements PivotIO {
    private final TalonFX motor = new TalonFX(PivotConstants.PIVOT_MOTOR_ID);
    private final CoastOut coastRequest = new CoastOut();
    private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0);
    private final MotionMagicTorqueCurrentFOC motionMagicRequest = new MotionMagicTorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

    StatusSignal <Angle> position = motor.getPosition();
    StatusSignal <AngularVelocity> velocity = motor.getVelocity();
    StatusSignal <Current> statorCurrent = motor.getStatorCurrent();
    StatusSignal <Current> supplyCurrent = motor.getSupplyCurrent();

    public PivotIOTalonFX() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.MotionMagic.MotionMagicCruiseVelocity = 10;
        conf.MotionMagic.MotionMagicAcceleration = 16;

        conf.CurrentLimits.SupplyCurrentLimit = 45;

        motor.getConfigurator().apply(conf);
    }

    @Override 
    public void updateInputs(PivotIOInputs inputs) {
        BaseStatusSignal.refreshAll(position, velocity, statorCurrent, supplyCurrent);

        inputs.pivotPosition = position.getValue();
        inputs.pivotVelocity = velocity.getValue();
        inputs.pivotStatorCurrent = statorCurrent.getValue();
        inputs.pivotSupplyCurrent = supplyCurrent.getValue();
    }

    @Override
    public void setVelocity(AngularVelocity velocity, int slot) {
        motor.setControl(velocityRequest.withVelocity(velocity).withSlot(slot));
    }

    @Override
    public void setPosition(Angle position, int slot) {
        motor.setControl(positionRequest.withPosition(position).withSlot(slot));
    }

    @Override
    public void setMotionMagicPosition(Angle position, int slot) {
        motor.setControl(motionMagicRequest.withPosition(position).withSlot(slot));
    }

    @Override
    public void coast() {
        motor.setControl(coastRequest);
    }

    @Override
    public void configurePID(double extensionP, double extensionI, double extensionD, double retractP, double retractI, double retractD) {

    TalonFXConfiguration conf = new TalonFXConfiguration();

        // For extension
        conf.Slot0.kP = extensionP;
        conf.Slot0.kD = extensionD;
        conf.Slot0.kI = extensionI;

        // Slot 1 is for contracting in the ramp command
        conf.Slot1.kP = retractP;
        conf.Slot1.kD = retractD;
        conf.Slot1.kI = retractI;

        // Slot 2 has a small PID for rezeroing
        conf.Slot2.kP = 5;

        conf.CurrentLimits.SupplyCurrentLimit = 45;

        motor.getConfigurator().apply(conf);
        }
}