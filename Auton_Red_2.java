package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.Auton_2;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpinnerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous
public class Auton_Red_2 extends CommandOpMode {

    private DriveSubsystem drive;
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private SpinnerSubsystem spinner;
    private Auton_2 command;
    private VisionSubsystem vision;

    @Override
    public void initialize() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        arm = new ArmSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, telemetry);

        command = new Auton_2("RED", drive, arm, intake, vision, telemetry);
        schedule(command);
        schedule(new RunCommand(() -> {
            telemetry.update();
        }));
    }
}
