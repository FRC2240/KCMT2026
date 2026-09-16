package frc.robot.subsystems.diagnostics;

import org.littletonrobotics.junction.AutoLog;

public interface DiagnosticsIO {
    @AutoLog
    public static class DiagnosticsIOInputs {
        // Add any inputs you want to log here
        public double BatteryVoltage = 0.0;
    }
    public default void updateInputs(DiagnosticsIOInputsAutoLogged inputs) {}
}