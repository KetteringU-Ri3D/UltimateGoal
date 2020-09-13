/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TankDrive", group="Linear Opmode")
//@Disabled
public class TankDrive extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        DcMotor indexerMotor = hardwareMap.get(DcMotor.class, "indexer");
        DcMotor shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
        DcMotor wobbleMotor = hardwareMap.get(DcMotor.class, "wobble");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
             * drive code
             */
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // arcade drive
            // double drive = -gamepad1.left_stick_y;
            // double turn = gamepad1.right_stick_x;
            // leftPower = Range.clip(drive + turn, -1.0, 1.0);
            // rightPower = Range.clip(drive - turn, -1.0, 1.0);

            // tank drive
            leftPower = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;

            leftFrontDrive.setPower(leftPower);
            leftRearDrive.setPower(leftPower);
            rightFrontDrive.setPower(rightPower);
            rightRearDrive.setPower(rightPower);

            /*
             * intake code
             */
            double intakePower = 0.5;
            if(gamepad1.right_trigger >= 0.2) {
                intakeMotor.setPower(intakePower);
            }
            else if(gamepad1.a) {
                intakeMotor.setPower(-intakePower);
            }

            /*
             * indexer code
             */
            double indexerPower = 0.5;
            if(gamepad1.left_trigger >= 0.2) {
                indexerMotor.setPower(indexerPower);
            }
            else if(gamepad1.left_bumper) {
                indexerMotor.setPower(-indexerPower);
            }

            /*
             * wobble goal code
             */
            double wobblePower = 0.5;
            if(gamepad1.y) {
                wobbleMotor.setPower(wobblePower);
            }
            else if(gamepad1.b) {
                wobbleMotor.setPower(-wobblePower);
            }

            /*
             * shooter code
             */
            double shooterPower = 0.5;
            if(buttonClick(gamepad1.right_bumper)) {
                shooterMotor.setPower(shooterPower);
            }
            else if(!buttonClick(gamepad1.right_bumper)) {
                shooterMotor.setPower(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }

    private boolean buttonPreviousState;
    public boolean buttonClick (boolean button) {
        boolean returnVal;
        returnVal = button && !buttonPreviousState;
        buttonPreviousState = button;
        return returnVal;
    }
}
