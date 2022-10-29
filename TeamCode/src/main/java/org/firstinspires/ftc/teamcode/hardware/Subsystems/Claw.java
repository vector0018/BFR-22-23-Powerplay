package org.firstinspires.ftc.teamcode.hardware.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw implements Subsystem{
    Servo leftClaw;
    Servo rightClaw;

    @Override
    public void initialize(HardwareMap map, Telemetry telemetry) {
        leftClaw = map.get(Servo.class, "LC");
        rightClaw = map.get(Servo.class, "RC");
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {

    }
    public void moveClaw(){
        //open claw
        if (gamepad2.left_bumper) {
            leftClaw.setPosition(.7);
        }
        if (gamepad2.left_bumper) {
            rightClaw.setPosition(.7);
        }
        //close claw
        if (gamepad2.right_bumper) {
            rightClaw.setPosition(0.5);
        }
        if (gamepad2.right_bumper) {
            leftClaw.setPosition(0.4);
        }
        // Open claw all; the way
        if (gamepad2.left_trigger > .1 || gamepad2.right_trigger > .1){
            leftClaw.setPosition(.9);
            rightClaw.setPosition(.9);
        }
    }
}
