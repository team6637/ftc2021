package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.Auton_1;
import org.firstinspires.ftc.teamcode.commands.DriveStraightCommand;
import org.firstinspires.ftc.teamcode.commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpinnerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous
public class Auton_Blue_1 extends CommandOpMode {

    private DriveSubsystem drive;
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private SpinnerSubsystem spinner;
    private Auton_1 command;
    private VisionSubsystem vision;

    @Override
    public void initialize() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        arm = new ArmSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        spinner = new SpinnerSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, telemetry);

        command = new Auton_1("BLUE", drive, arm, intake, spinner, vision, telemetry);
        schedule(command);
        schedule(new RunCommand(()->{ telemetry.update();}));
    }
}
