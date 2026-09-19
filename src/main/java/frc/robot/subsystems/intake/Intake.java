package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private String state = "None";

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        inputs.state = state;

        Logger.processInputs("Intake", inputs);
    }

    public void setVelocity(AngularVelocity velocity) {
        io.setVelocity(velocity);
    }

    public Command setIntakeVelocityCommand(AngularVelocity velocity) {
        return runOnce(() -> setVelocity(velocity));
    }

    public Command setIntakeVelocityCommand(Supplier<AngularVelocity> velocity) {
        return runOnce(() -> setVelocity(velocity.get()));
    }

   /*  public Command enableIntakeCommand() {
        return Commands.repeatingSequence(
                run(() -> {
                    double MAX_ROBOT_SPEED = 4.5;
                    double MIN_ROLLER_SPEED = 50;
                    double MAX_ROLLER_SPEED = 90;
                    ChassisSpeeds speeds = RobotPosition.getChassisSpeeds();


                    double speed = Math
                            .sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
                    speed = Math.min(speed, MAX_ROBOT_SPEED); // Caps to 4.5 m/s
                    double fraction = speed / MAX_ROBOT_SPEED;
                    double desiredRollerSpeed = MIN_ROLLER_SPEED + (MAX_ROLLER_SPEED - MIN_ROLLER_SPEED) * fraction;
                    SmartDashboard.putNumber("Intake/Desired Roller Speed", -desiredRollerSpeed);

                    io.setVelocity((RotationsPerSecond.of(-desiredRollerSpeed)));
                }));
    }
*/ 
// commented out until drivetrain/utils are in the code
 
    public Command disableIntakeCommand() {
        return setIntakeVelocityCommand(RotationsPerSecond.of(0)).andThen(run(() -> {
            state = "Disable";
        })).withName("Disable");
    }

    public Command reverseIntakeCommand() {
        return setIntakeVelocityCommand(IntakeIOConstants.INTAKE_VELOCITY.unaryMinus()).andThen(run(() -> {
            state = "Reverse";
        })).withName("Reverse");
    }

    public Command slowReverseIntakeCommand() {
        return setIntakeVelocityCommand(RotationsPerSecond.of(1)).andThen(run(() -> {
            state = "Slow";
        }));
    }

    public Command ReverseIntakeCommand() {
        return runOnce(() -> setIntakeVelocityCommand(RotationsPerSecond.of(-1)));
    }
}

