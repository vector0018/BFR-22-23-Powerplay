package org.firstinspires.ftc.teamcode.hardware.Subsystems;

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
}
