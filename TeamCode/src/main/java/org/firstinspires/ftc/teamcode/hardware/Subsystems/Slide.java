package org.firstinspires.ftc.teamcode.hardware.Subsystems;

import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.maxTargetPosition;
import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.minTargetPosition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide implements Subsystem {

    DcMotor slideMotor;
    double currentPosition;
    double zeroPos;
    double slidePower;

    public Slide(HardwareMap map){

    }

    @Override
    public void initialize(HardwareMap map, Telemetry telemetry) {
        slideMotor = map.get(DcMotor.class, "SM");
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        zeroPos = encoderTicksToInches(slideMotor.getCurrentPosition());

    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {

    }

    public void moveSlide(double targetPosition, double runTime) {
        double Kp = 0.2;
        double positionError;
        ElapsedTime timer = new ElapsedTime();

        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        timer.reset();
        while (Math.abs(currentPosition - targetPosition) < 0.15 && timer.seconds() < runTime) {

            //safeties for slide pos
            if (targetPosition > maxTargetPosition) {
                targetPosition = maxTargetPosition;
            }
            if (targetPosition < minTargetPosition) {
                targetPosition = minTargetPosition;
            }
            //takes starting position and subtracts from current input to get real position
            positionError = targetPosition - currentPosition;

            //proportional pid controller
            slidePower = Kp * positionError;

            //power safeties
            if (slidePower > 1) {
                slidePower = 1;
            }
            if (slidePower < -1) {
                slidePower = -1;
            }
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
    }

        public void moveSlideWithoutTime(double targetPosition2) {
            double Kp2 = 0.2;
            double positionError2;

            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;

            //safeties for slide pos
            if (targetPosition2 > maxTargetPosition) {
                targetPosition2 = maxTargetPosition;
            }
            if (targetPosition2 < minTargetPosition) {
                targetPosition2 = minTargetPosition;
            }
            //takes starting position and subtracts from current input to get real position
            positionError2 = targetPosition2 - currentPosition;

            //proportional pid controller
            slidePower = Kp2 * positionError2;

            //power safeties
            if (slidePower > 1) {
                slidePower = 1;
            }
            if (slidePower < -1) {
                slidePower = -1;
            }
            slideMotor.setPower(slidePower);
    }
}
