package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AutonTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        init();
        DcMotor backLeftMotor = null;
        DcMotor backRightMotor = null;
        DcMotor frontRightMotor = null;
        DcMotor frontLeftMotor = null;
        waitForStart();
    }
}
