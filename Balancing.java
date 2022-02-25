package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Balancing", group="Linear Opmode")
public class Balancing extends BalancingRobot{

    double targetAngle = -90.0;
    double currentAngle = 0.0;

    double forwardP = 0.1;
    double backwardP = 0.05;

    double speed;
    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware Initialization
        hardware.init(hardwareMap);

        // Ready For Game Start
        telemetry.addData("Status", "Ready For Start");
        telemetry.update();

        // Wait for the game start
        waitForStart();

        while (opModeIsActive()){
            currentAngle = getCurrentAngleX();

            if (currentAngle < -90) {
                speed = -(currentAngle - targetAngle);
                speed *= forwardP;

                hardware.LeftDrive.setPower(speed);
                hardware.RightDrive.setPower(speed);
            }
            else if (currentAngle > -90) {
                speed = (currentAngle - targetAngle);
                speed *= backwardP;

                hardware.LeftDrive.setPower(-speed);
                hardware.RightDrive.setPower(-speed);
            }
        }
    }
}
