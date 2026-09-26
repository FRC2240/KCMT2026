package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TunableNumber;

public class Pivot extends SubsystemBase {
    private PivotIO io;
    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private String state = "None";

    private TunableNumber extensionP = new TunableNumber("Pivot/PID/extension P", 25);
    private TunableNumber extensionI = new TunableNumber("Pivot/PID/extension I", 7);
    private TunableNumber extensionD = new TunableNumber("Pivot/PID/extension D", 10);

    private TunableNumber retractP = new TunableNumber("Pivot/PID/retract P", 10);
    private TunableNumber retractI = new TunableNumber("Pivot/PID/retract I", 3);
    private TunableNumber retractD = new TunableNumber("Pivot/PID/retract D", 4);

    public Pivot(PivotIO io) {
        this.io = io;

        configurePID();
        extensionP.addChangeListener((v) -> configurePID());
        extensionI.addChangeListener((v) -> configurePID());
        extensionD.addChangeListener((v) -> configurePID());

        retractP.addChangeListener((v) -> configurePID());
        retractI.addChangeListener((v) -> configurePID());
        retractD.addChangeListener((v) -> configurePID());

        io.setPosition(Rotations.of(0),0); //Everything is slot 0 for now because
    }                                                     //idk PID slots want to be used

    private void configurePID() {
        io.configurePID(extensionP.get(), extensionI.get(), 
            extensionD.get(), retractP.get(), 
            retractI.get(), retractD.get());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        Logger.recordOutput("State", state);
    }

    public Command setPositionCommand(Angle position) {
        return runOnce(() -> {
            state = "Set Position";
            io.setPosition(position,0);
        });
    }

    public void extendMotionMagic() {
        state = "Extend Motion Magic";
        io.setMotionMagicPosition(PivotConstants.PIVOT_EXTENDED_POSITION,0);
    }

    public void extend() {
        state = "Extend";
        io.setPosition(PivotConstants.PIVOT_EXTENDED_POSITION,0);
    }

    public void ramp() {
        state = "Ramp";
        io.setPosition(PivotConstants.PIVOT_RAMP_POSITION, 0);
    }

    public void rampBottom() {
        state = "Ramp Bottom";
        io.setPosition(PivotConstants.PIVOT_RAMP_BOTTOM_POSITION, 0);
    }

    public Command testRamp() {
        return runOnce(this::ramp);
    }

    public void coast() {
        state = "Coast";
        io.coast();
    }

    public Command extendCommand() {
        return runOnce(this::extend).withName("Extend");
    }

    public Command extendRunCommand() {
        return extendCommand()
                .andThen(run(() -> {}));
    }

    public Command contractCommand() {
        return runOnce(() -> {
            io.setMotionMagicPosition(Rotations.of(0),0);
            state = "Contract";
        })
                .andThen(run(() -> {
                })).withName("Contract");
    }

    public Command rezeroCommand() {
        return Commands.sequence(
                runOnce(() -> {
                    state = "Rezero";
                    io.setVelocity(RotationsPerSecond.of(-2), 2); 
                }),
                Commands.waitSeconds(0.3),
                Commands.waitUntil(() -> Math.abs(inputs.pivotVelocity.in(RotationsPerSecond)) < 0.2),
                runOnce(() -> io.setPosition(PivotConstants.PIVOT_EXTENDED_POSITION,0)),
                extendCommand())
                .withName("Rezero");
    }
    

    public Command rampCommand() {
        return Commands.repeatingSequence(
                runOnce(this::ramp),
                Commands.waitSeconds(1.1),
                runOnce(this::extend),
                Commands.waitSeconds(1.1));
    }

    /**
     * Useful for tuning PID values from Shuffleboard/Elastic.
     */
    public Command setPositionCommand(Supplier<Angle> position, int slot) {
        return run(() -> io.setPosition(position.get(), slot));
    }
}
