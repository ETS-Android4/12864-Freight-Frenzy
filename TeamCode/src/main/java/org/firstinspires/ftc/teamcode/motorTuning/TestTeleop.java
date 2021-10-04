package org.firstinspires.ftc.teamcode.motorTuning;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="PID testing")
public class TestTeleop extends CommandOpMode {

    private Motor testMotor;
    private GamepadEx driver;

    private MotorPIDSubsystem subsystem;
    private MotorPIDCommand command;

    @Override
    public void initialize() {
        testMotor = new Motor(hardwareMap, "motor");

        driver = new GamepadEx(gamepad1);

        subsystem = new MotorPIDSubsystem(testMotor, telemetry);
        command = new MotorPIDCommand(subsystem);

        driver.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(command);
    }
}
