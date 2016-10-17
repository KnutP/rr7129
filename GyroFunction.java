    package org.firstinspires.ftc.teamcode;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.GyroSensor;

    public class GyroFunction extends LinearOpMode {

        DcMotor lDrive1;
        DcMotor lDrive2;
        DcMotor rDrive1;
        DcMotor rDrive2;

        GyroSensor gyroSensor;

        // Function to set up the Gyro
        // Function called in the init
        // Calibrates and does other preparations for the gyro sensor before autonomous
        // Needs nothing passed to it
        private void setUpGyro() throws InterruptedException {

            // setup the Gyro
            // write some device information (connection info, name and type)
            // to the log file.
            hardwareMap.logDevices();
            // get a reference to our GyroSensor object.
            gyroSensor = hardwareMap.gyroSensor.get("gyro");
            // calibrate the gyro.
            gyroSensor.calibrate();
            while (gyroSensor.isCalibrating())  {
                sleep(50);
            }
            // End of setting up Gyro
        }


        // Function to use the gyro to do a spinning turn in place.
        // It points the robot at an absolute heading, not a relative turn.  0 will point robot to same
        // direction we were at the start of program.
        // Pass:
        // targetHeading = the new heading we want to point robot at,
        // maxSpeed = the max speed the motor can run in the range of 0 to 1
        // direction = the direction we will turn, 1 is clockwise, -1 is counter-clockwise
        // Returns:
        // heading = the new heading the gyro reports
        public int gyroTurn(int targetHeading, double maxSpeed, int direction) {
            int startHeading = gyroSensor.getHeading();
            int error;
            int currentHeading;
            double oldSpeed;
            double speed = 0;
            double minSpeed = 0.05;
            double acceleration = 0.01;
            double kp = 0.01;             // Proportional error constant

            lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Calls turnCompeted function to determine if we need to keep turning
            while ((!turnCompleted(gyroSensor.getHeading(), targetHeading, 5, direction) && opModeIsActive())) {

                // Calculate the speed we should be moving at
                oldSpeed = speed;           // save our old speed for use later.

                currentHeading = gyroSensor.getHeading();
                // Reuses the degreesToTurn function by passing different values to obtain our error
                error = degreesToTurn(targetHeading, currentHeading, direction);

                speed = error * kp * direction;

                // Limit the acceleration of the motors speed at beginning of turns.
                if( Math.abs(speed) > Math.abs(oldSpeed))
                    speed = oldSpeed + (direction * acceleration);

                // Set a minimum power for the motors to make sure they move
                if (Math.abs(speed) < minSpeed)
                    speed = minSpeed * direction;


                // Don't exceed the maximium speed requested
                if (Math.abs(speed) > maxSpeed)
                    speed = maxSpeed * direction;

                // Set the motor speeds
                lDrive1.setPower(speed);
                lDrive2.setPower(speed);
                rDrive1.setPower(-speed);
                rDrive2.setPower(-speed);

            }
            // Done with the turn so shut off motors
            lDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive1.setPower(0);
            rDrive2.setPower(0);
            telemetry.addData("1. h", startHeading);
            telemetry.addData("2. g", gyroSensor.getHeading());

            return (gyroSensor.getHeading());
        }

        // Function used by the turnCompleted function (which is used by the gyroTurn function) to determine the degrees to turn.
        // Also used to determine error for proportional speed control in the gyroTurn function
        // by passing targetHeading and currentHeading instead of startHeading and targetHeading, respectively
        // Pass:
        // startHeading = heading that we are at before we turn
        // targetHeading = heading we want to turn to
        // direction = the direction we want to turn (1 for right, -1 for left)
        public static int degreesToTurn(int startHeading, int targetHeading, int direction) {

            int degreesToTurn;

            // Turning right
            if (direction == 1)
                degreesToTurn = targetHeading - startHeading;
            // Turning left
            else
                degreesToTurn = startHeading - targetHeading;

            while (degreesToTurn < 0)
                degreesToTurn = degreesToTurn + 360;
            while (degreesToTurn > 360)
                degreesToTurn = degreesToTurn + 360;

            return (degreesToTurn);
        }

        // Function used by the GyroTurn function to determine if we have turned far enough.
        // Pulls from degreesToTurn function to determine degrees to turn.
        // Pass:
        // currentHeading = the heading the robot is currently at (live value, changes during the turn)
        // targetHeading = the heading we want the robot to be at once the turn is completed
        // range = the degrees of error we are allowing so that the robot doesn't spin in circles
        // if it overshoots by a small amount
        // direction = direction the robot is turning (1 for right, -1 for left)
        // Returns:
        // result = true if we have reached target heading, false if we haven't
        public static boolean turnCompleted(int currentHeading, int targetHeading, int range, int direction)  {

            // Value we want to stop at
            int stop;
            // Will be sent to the GyroTurn function; robot stops turning when true
            boolean result;

            // Turn right
            if (direction >= 1) {
                stop = targetHeading + range;
                if (stop <= 359 )
                    result = (currentHeading >= targetHeading && currentHeading <= stop);
                else // We wrapped around from 359 to 0 degrees
                    result = (currentHeading >= targetHeading && currentHeading <= 359) || (currentHeading >=0 && currentHeading <= (stop-360));
            }
            // Turn left
            else {
                stop = targetHeading - range;
                if (stop >= 0 )
                    result = (currentHeading <= targetHeading && currentHeading >= stop);
                else // We wrapped around from 0 to 359 degrees
                    result = ((currentHeading >= (stop+360)  && currentHeading <=359) || (currentHeading >= 0 && currentHeading <= targetHeading));
            }
            return (result);
        }



        public void runOpMode() throws InterruptedException {

            lDrive1 = hardwareMap.dcMotor.get("lDrive1");
            lDrive2 = hardwareMap.dcMotor.get("lDrive2");
            rDrive1 = hardwareMap.dcMotor.get("rDrive1");
            rDrive2 = hardwareMap.dcMotor.get("rDrive2");
            rDrive1.setDirection(DcMotor.Direction.REVERSE);
            rDrive2.setDirection(DcMotor.Direction.REVERSE);

            setUpGyro();

            lDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            waitForStart();

            gyroTurn(45,1,1);


        }
    }
