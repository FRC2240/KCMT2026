package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {

        public AngularVelocity LeftUpperFlywheelVelocity;
        public AngularVelocity LeftLowerFlywheelVelocity;
        public AngularVelocity RightUpperFlywheelVelocity;
        public AngularVelocity RightLowerFlywheelVelocity;

        public Current LeftUpperFlywheelStatorCurrent;
        public Current LeftLowerFlywheelStatorCurrent;
        public Current RightUpperFlywheelStatorCurrent;
        public Current RightLowerFlywheelStatorCurrent;

        public Current LeftUpperFlywheelSupplyCurrent;
        public Current LeftLowerFlywheelSupplyCurrent;
        public Current RightUpperFlywheelSupplyCurrent;
        public Current RightLowerFlywheelSupplyCurrent;
    }
    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVelocity(AngularVelocity velocity) {}
}
