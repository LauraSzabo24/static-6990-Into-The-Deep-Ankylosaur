package org.firstinspires.ftc.teamcode.drive.PinThings

import com.acmerobotics.roadrunner.geometry.Pose2d
/**
 * Generic interface for estimating robot pose over time.
 */
interface pinLocal {

    /**
     * Current robot pose estimate.
     */
    var poseEstimate: Pose2d

    /**
     * Current robot pose velocity (optional)
     */
    val poseVelocity: Pose2d?

    /**
     * Completes a single localization update.
     */
    fun update()
}
