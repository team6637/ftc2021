package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveStraightCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpinnerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp
public class OurTeleOp extends CommandOpMode {

	private DriveSubsystem drive;
	private ArmSubsystem arm;
	private IntakeSubsystem intake;
	private GamepadEx gamepad;
	private SpinnerSubsystem spinner;
	private VisionSubsystem vision;

	@Override
	public void initialize() {
		drive = new DriveSubsystem(hardwareMap, telemetry);
		arm = new ArmSubsystem(hardwareMap);
		intake = new IntakeSubsystem(hardwareMap);
		gamepad = new GamepadEx(gamepad1);
		spinner = new SpinnerSubsystem(hardwareMap);
		vision = new VisionSubsystem(hardwareMap, telemetry);

		drive.setDefaultCommand(new DriveCommand(drive, gamepad));

		// INTAKE COMMAND
		schedule(
			new RunCommand(() -> {
				if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
					intake.move(1.0);
					intake.closeFlapper();

					if (intake.getSensorSeesObject()) {
						arm.setState("FORWARD_BOTTOM");
						//intake.flapperHoldObject();
					}
				} else if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
					if (arm.getState() == "FLOOR") {
						intake.move(-1.0);
					} else {
						intake.move(-0.2);
						intake.openFlapper();
					}
				} else {
					intake.move(0.0);
				}
			})
		);


		// LEFT BUMPER:
		// Set Drive: BACKWARD
		// Set Arm: STRAIGHT_UP
//		gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//			new ConditionalCommand(
//				new ParallelCommandGroup(
//					new InstantCommand(() -> {
//						drive.setState("BACKWARD");
//					}),
//					new InstantCommand(() -> {
//						arm.setState("BACKWARD_TOP");
//					})
//				),
//				new InstantCommand(() -> {
//				}),
//				() -> drive.getState() != "BACKWARD"
//			)
//		);

		// RIGHT BUMPER:
		// Set Drive: FORWARD
		// Set Arm: STRAIGHT_UP
//		gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
//			new ConditionalCommand(
//				new ParallelCommandGroup(
//					new InstantCommand(() -> {
//						drive.setState("FORWARD");
//					}),
//					new InstantCommand(() -> {
//						arm.setState("FORWARD_TOP");
//					})
//				),
//				new InstantCommand(() -> {
//				}),
//				() -> drive.getState() != "FORWARD"
//			)
//		);

		// A BUTTON:
		// Set Arm Position
		gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
			new ConditionalCommand(
				new SequentialCommandGroup(
					new RunCommand(() -> {
						intake.closeFlapper();
					}).withTimeout(200),
					new InstantCommand(() -> {
						arm.setState("FLOOR");
					})
				),
				new InstantCommand(() -> {
					arm.setState("BACKWARD_MIDDLE");
				}),
				() -> drive.getState() == "FORWARD"
			)
		);

		// X BUTTON:
		// Set Arm Position
		gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
			new ConditionalCommand(
				new InstantCommand(() -> {
					arm.setState("FORWARD_BOTTOM");
				}),
				new InstantCommand(() -> {
					arm.setState("BACKWARD_TOP");
				}),
				() -> drive.getState() == "FORWARD"
			)
		);

		// Y BUTTON:
		// Set Arm Position
		gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
			new ConditionalCommand(
				new InstantCommand(() -> {
					arm.setState("FORWARD_MIDDLE");
				}),
				new InstantCommand(() -> {
				}),
				() -> drive.getState() == "FORWARD"
			)
		);

		// B BUTTON:
		// Set Arm Position
		gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
			new ConditionalCommand(
				new InstantCommand(() -> {
					arm.setState("FORWARD_TOP");
				}),
				new InstantCommand(() -> {
				}),
				() -> drive.getState() == "FORWARD"
			)
		);

		gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileHeld(
			new InstantCommand(() -> {
				spinner.move();
			})
		).whenReleased(
			new InstantCommand(() -> {
				spinner.stop();
			})
		);
	}
}