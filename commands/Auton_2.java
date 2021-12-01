package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpinnerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Config
public class Auton_2 extends SequentialCommandGroup {

    private int inversionMultiplier = 1;
    private int position;

    public static double AUTON2_FIRST_DISTANCE = 30.0;
    public static double AUTON2_FIRSTB_DISTANCE = -14.0;
    public static double AUTON2_FIRST_ANGLE = -84.0;
    public static double AUTON2_SECOND_DISTANCE = -18.0;
    public static double AUTON2_SECOND_ANGLE = 84.0;
    public static double AUTON2_THIRD_DISTANCE = 12.0;
    public static double AUTON2_FOURTH_DISTANCE = -6.0;
    public static double AUTON2_THIRD_ANGLE = -84.0;
    public static double AUTON2_FIFTH_DISTANCE = 100.0;

    public Auton_2(String alliance, DriveSubsystem drive, ArmSubsystem arm,
                   IntakeSubsystem intake, VisionSubsystem vision,
                   Telemetry telemetry){
        if(alliance == "BLUE") inversionMultiplier = -1;

        addCommands(
            new WaitUntilCommand(vision::positionIsSet).withTimeout(5000),
            new InstantCommand(()->{ intake.closeFlapper();}),
            new InstantCommand(()->{ position = vision.getPosition();}),
            new InstantCommand(()->{
                if(position == 1) {
                    arm.setState("FORWARD_BOTTOM");
                } else if(position == 2) {
                    arm.setState("FORWARD_MIDDLE");
                } else {
                    arm.setState("FORWARD_TOP");
                }

            }),
            new DriveStraightCommand(drive, AUTON2_FIRST_DISTANCE, telemetry).withTimeout(3000),
            new DriveStraightCommand(drive, AUTON2_FIRSTB_DISTANCE, telemetry).withTimeout(3000),
            new TurnToAngle(drive, AUTON2_FIRST_ANGLE * inversionMultiplier, telemetry).withTimeout(3000),
            new DriveStraightCommand(drive,AUTON2_SECOND_DISTANCE, telemetry).withTimeout(3000),
            new TurnToAngle(drive, AUTON2_SECOND_ANGLE * inversionMultiplier, telemetry).withTimeout(3000),
            new DriveStraightCommand(drive,AUTON2_THIRD_DISTANCE, telemetry).withTimeout(3000),
            new RunCommand(()->{
                intake.move(-0.2);
                intake.openFlapper();
            }).withTimeout(2000),
            new ParallelCommandGroup(
                    new DriveStraightCommand(drive, AUTON2_FOURTH_DISTANCE, telemetry).withTimeout(3000),
                    new RunCommand(()->{
                        intake.closeFlapper();
                        intake.move(0.0);
                    }).withTimeout(1000)
            ),
            new TurnToAngle(drive, AUTON2_THIRD_ANGLE * inversionMultiplier, telemetry).withTimeout(3000),
            new InstantCommand(()->{ drive.setAutonMaxPower(0.95);}),
            new DriveStraightCommand(drive, AUTON2_FIFTH_DISTANCE, telemetry).withTimeout(6000),
            new InstantCommand(()-> {
                arm.setState("FLOOR");
            })

        );
        //addRequirements(drive, arm, intake, spinner, vision);
    }
}