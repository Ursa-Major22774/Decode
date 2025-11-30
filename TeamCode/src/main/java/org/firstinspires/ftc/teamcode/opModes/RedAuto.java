package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Red Auto", group = "Competition Autos")
@Configurable
public class RedAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, opmodeTimer;
    private int pathState = 1; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Intake intake;
//    private Transfer transfer;
//    private Turret turret;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(87.726, 8.194, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        intake = new Intake(hardwareMap);
//        transfer = new Transfer(hardwareMap);
//        turret = new Turret(hardwareMap);


    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    @Configurable
    public static class Paths {
        public static PathChain Path1;
        public static PathChain Path2;
        public static PathChain Path3;
        public static PathChain Path4;
        public static PathChain Path5;
        public static PathChain Path6;
        public static PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(87.726, 8.194), new Pose(103.511, 35.307))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(103.511, 35.307), new Pose(129.660, 36.030))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(129.660, 36.030), new Pose(99.896, 7.592))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(99.896, 7.592), new Pose(103.391, 59.528))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(103.391, 59.528), new Pose(130.142, 59.649))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(130.142, 59.649), new Pose(89.533, 76.037))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(89.533, 76.037), new Pose(101.488, 82.567))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch(pathState) {
            case 1:
                follower.followPath(paths.Path1);
                setPathState(pathState + 1);
                break;
            case 2:
                if (!follower.isBusy()){
                    intake.intake();
                    follower.followPath(paths.Path2);
                    setPathState(pathState + 1);
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    intake.stop();
                    follower.followPath(paths.Path3);
                    setPathState(pathState + 1);
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    follower.followPath(paths.Path4);
                    setPathState(pathState + 1);
                }
                break;
            case 5:
                if (!follower.isBusy()){
                    intake.intake();
                    follower.followPath(paths.Path5);
                    setPathState(pathState + 1);
                }
                break;
            case 6:
                if (!follower.isBusy()){
                    intake.stop();
                    follower.followPath(paths.Path6);
                    setPathState(pathState + 1);
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    follower.followPath(paths.Path7);
                    setPathState(pathState + 1);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    pathState = 0;
                }
                break;
        }

        return pathState;
    }

    public void setPathState (int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }
}
