package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Tuning", group="Linear Opmode")
public class Tuning extends BalancingRobot {

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware Initialization
        hardware.init(hardwareMap);

        // Ready For Game Start
        telemetry.addData("Status", "Ready For Start");
        telemetry.update();

        // Wait for the game start
        waitForStart();

        ///////////////GAME START////////////////////////////////

        while (opModeIsActive()) {


            // Angle
            telemetry.addData("Z: ", getCurrentAngleZ());
            telemetry.addData("X: ", getCurrentAngleX());
            telemetry.addData("Y: ", getCurrentAngleY());

            hardware.LeftDrive.setPower(0.5);
            hardware.RightDrive.setPower(0.5);


            telemetry.update();
        }
    }
}