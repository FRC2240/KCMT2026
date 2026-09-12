package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {

        public AngularVelocity intakeSpinVelocity;
        public Current intakeStatorCurrent;
        public Current intakeSupplyCurrent;

        public AngularVelocity followerSpinVelocity;
        public Current followerStatorCurrent;
        public Current followerSupplyCurrent;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setVelocity(AngularVelocity velocity) {}   
}
