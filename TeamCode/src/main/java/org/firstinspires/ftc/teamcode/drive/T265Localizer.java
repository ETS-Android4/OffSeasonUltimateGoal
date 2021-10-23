package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class T265Localizer implements Localizer {

    private Pose2d poseOffset = new Pose2d();
    private static Pose2d mPoseEstimate = new Pose2d();
    private Pose2d rawPose = new Pose2d();

    public static T265Camera slamra;
    private T265Camera.CameraUpdate up;
    private static T265Camera.PoseConfidence poseConfidence;

    public T265Localizer(HardwareMap hardwareMap) {
        new T265Localizer(hardwareMap, true);
    }

    public T265Localizer(HardwareMap hardwareMap, boolean resetPos) {
        poseOffset = new Pose2d();
        mPoseEstimate = new Pose2d();
        rawPose = new Pose2d();

        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(new Translation2d(0, 0), new Rotation2d(0)), 0, hardwareMap.appContext);
            RobotLog.d("T265 object created");
            setPoseEstimate(new Pose2d(0, 0, 0));
        }
        try {
            startT265();
        }
        catch (Exception ignored){
            RobotLog.v("T265 already started");
            if (resetPos) {
                slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(0, 0, new Rotation2d(0)));
            }
        }
        if (slamra.getLastReceivedCameraUpdate().confidence == T265Camera.PoseConfidence.Failed) {
            RobotLog.e("Pose Confidence Failed");
        }
    }
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        if (up != null) {
            Translation2d pos = up.pose.getTranslation();
            Rotation2d rot = up.pose.getRotation();
            rawPose = new Pose2d(pos.getX() / 0.0254, pos.getY() / 0.0254, rot.getRadians());
            mPoseEstimate = rawPose.plus(poseOffset);
        }
        else {
            RobotLog.v("CameraUpdate is null");
        }

        return mPoseEstimate;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        pose2d = new Pose2d(pose2d.getX(), pose2d.getY(), 0);
        poseOffset = pose2d.minus(rawPose);
        poseOffset = new Pose2d(poseOffset.getX(), poseOffset.getY(), Math.toRadians(0));
        mPoseEstimate = rawPose.plus(poseOffset);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        ChassisSpeeds velo = up.velocity;
        Pose2d poseVelo = new Pose2d(velo.vxMetersPerSecond / 0.0254, velo.vyMetersPerSecond / 0.0254, velo.omegaRadiansPerSecond);
        return poseVelo;
    }

    public static T265Camera.PoseConfidence getConfidence() {
        return poseConfidence;
    }

    public static double getHeading() {
        return mPoseEstimate.getHeading();
    }

    @Override
    public void update() {
        up = slamra.getLastReceivedCameraUpdate();
        poseConfidence = up.confidence;
    }

    public void startT265() {
        slamra.start();
    }

    public void stopT265() {
        slamra.stop();
    }
}
