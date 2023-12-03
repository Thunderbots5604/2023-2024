package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mecanumdrive.DriveTrain;
import org.firstinspires.ftc.teamcode.robotParts.Arm;

import org.firstinspires.ftc.teamcode.robotParts.Claw;
import org.firstinspires.ftc.teamcode.robotParts.Intake;

@TeleOp(name = "Alpha", group = "competition")
public class TeleOpAlpha extends OpMode {
    DriveTrain drive;
    Arm arm;
//    Claw claw;
    Intake intake;

    boolean doneInit = false;

    boolean pastBack = false;
    boolean currentBack;
    boolean pastA = false;
    boolean currentA;
    boolean pastB = false;
    boolean currentB;
    boolean pastD_Up = false;
    boolean currentD_Up;
    boolean pastD_Down = false;
    boolean currentD_Down;
    boolean pastD_Right = false;
    boolean currentD_Right;
    boolean halfSpeed;
    double multiplier;

    boolean pastTriggered = false;
    boolean currentRightTrigger = false;
    boolean currentLeftTrigger = false;
    double rightTriggerValue = 0;
    double leftTriggerValue = 0;

    @Override
    public void start() {
        while(!doneInit){}
//        claw.close();
        intake.setState("off");
    }

    @Override
    public void init() {
        drive = new DriveTrain(hardwareMap, "fl", "fr", "bl", "br");
//        arm = new Arm(hardwareMap, "am1", "am2", 10, 1100, 0.05);
//        claw = new Claw(hardwareMap, "claw", 50, 250);
        intake = new Intake(hardwareMap, "intMotor");
        doneInit = true;
    }

    @Override
    public void loop() {

        currentB = gamepad1.b;
        if(currentB && !pastB) {
            halfSpeed = !halfSpeed;
        }
        pastB = currentB;

        multiplier = halfSpeed ? 0.25 : 0.5;

        drive.setPowersToZero();
        drive.calculatePower(new double[] {multiplier * gamepad1.left_stick_x, -multiplier * gamepad1.left_stick_y, multiplier * gamepad1.right_stick_x});
        drive.normalizePowers();
        drive.pushPowers();

        currentD_Up = gamepad1.dpad_up;
        if(currentD_Up && !pastD_Up) {
            intake.setState("outputting");
        }
        pastD_Up = currentD_Up;

        currentD_Down = gamepad1.dpad_down;
        if(currentD_Down && !pastD_Down) {
            intake.setState("intaking");
        }
        pastD_Down = currentD_Down;

        currentD_Right = gamepad1.dpad_right;
        if(currentD_Right && !pastD_Right) {
            intake.setState("off");
        }
        pastD_Right = currentD_Right;
//
//        currentA = gamepad1.a;
//        if(currentA && !pastA) {
//            claw.toggle();
//        }
//        pastA = currentA;

        rightTriggerValue = gamepad1.right_trigger;
        leftTriggerValue = gamepad1.left_trigger;
        if(rightTriggerValue > 0.5) {
            currentRightTrigger = true;
        }
        else if(leftTriggerValue > 0.5) {
            currentLeftTrigger = true;
        }

//        if(currentRightTrigger) {
//            arm.movePower(true);
//        }
//        else if(currentLeftTrigger) {
//            arm.movePower(false);
//        }
//
//        if(!(currentLeftTrigger || currentRightTrigger)) {
//            if(!pastTriggered) {
//                arm.stop();
//            }
//        }

        pastTriggered = currentRightTrigger || currentLeftTrigger;
        currentRightTrigger = false;
        currentLeftTrigger = false;
    }

    @Override
    public void stop() {
        drive.stop();
//        arm.stop();
        intake.setState("off");
    }
}
