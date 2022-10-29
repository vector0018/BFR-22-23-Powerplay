package org.firstinspires.ftc.teamcode.hardware.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw implements Subsystem{
    Servo leftClaw;
    Servo rightClaw;

    public Claw(HardwareMap map){

    }

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
    public void moveClaw(boolean close, boolean open, double fullOpenLeft, double fullOpenRight){
        //open claw
        if (open) {
            leftClaw.setPosition(.7);
        }
        if (open) {
            rightClaw.setPosition(.7);
        }
        //close claw
        if (close) {
            rightClaw.setPosition(0.5);
        }
        if (close) {
            leftClaw.setPosition(0.4);
        }
        // Open claw all; the way
        if (fullOpenLeft > .1 || fullOpenRight > .1){
            leftClaw.setPosition(.9);
            rightClaw.setPosition(.9);
        }
    }

    public void openClaw(){
        leftClaw.setPosition(.7);
        rightClaw.setPosition(.7);
    }

    public void closeClaw(){
        rightClaw.setPosition(0.5);
        leftClaw.setPosition(0.4);
    }

    public void fullyOpenClaw(){
        leftClaw.setPosition(.9);
        rightClaw.setPosition(.9);
    }
}
