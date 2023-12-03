package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

public class Intake {
    private DcMotor motor;
    private String state = "off";

    public Intake(HardwareMap map, String name) {
        motor = map.get(DcMotor.class, name);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setState(String newState){
        if(Objects.equals(newState, "intaking")){
            motor.setPower(0.3);
            state = "intaking";
        } else if(Objects.equals(newState, "outputting")){
            motor.setPower(-0.3);
            state = "outputting";
        } else if(Objects.equals(newState, "off")){
            motor.setPower(0);
            state = "off";
        }
    }
}
