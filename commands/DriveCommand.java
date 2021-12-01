package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private DriveSubsystem drive;
    private GamepadEx gamepad;
    private double power;
    private double turn;

    public DriveCommand(DriveSubsystem drive, GamepadEx gamepad) {
        this.drive = drive;
        this.gamepad = gamepad;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        power = gamepad.getLeftY();
        turn = gamepad.getRightX();
        drive.manualDrive(power, turn);
    }
}
