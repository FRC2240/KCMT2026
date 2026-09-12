package frc.robot.subsystems.intakeRoller;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public interface IntakeRollerIO {

    @AutoLog
    public static class IntakeRollerIOInputs {

        public AngularVelocity intakeSpinVelocity;
        public Current intakeStatorCurrent;
        public Current intakeSupplyCurrent;

        public AngularVelocity followerSpinVelocity;
        public Current followerStatorCurrent;
        public Current followerSupplyCurrent;
    }

    public default void updateInputs(IntakeRollerIOInputs inputs) {}

    public default void setVelocity(AngularVelocity velocity) {}   
}
