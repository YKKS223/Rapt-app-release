package com.example.test_gyro_1.processor

import android.graphics.PointF
import com.example.test_gyro_1.gps.LocationData

data class MotionState(
    val position: FloatArray = FloatArray(3),
    val velocity: FloatArray = FloatArray(3),
    val worldAccel: FloatArray = FloatArray(3),

    // ユーザー定義の軸マッピング: Pitch(X), Roll(Y), Yaw(Z)　大事
    // 相対角度 (初期向きからの差分)
    val relativePitch: Float = 0f, // Pitch (X軸周りの相対回転)
    val relativeRoll: Float = 0f,  // Roll (Y軸周りの相対回転)
    val relativeYaw: Float = 0f,   // Yaw (Z軸周りの相対回転)

    // 絶対角度 (ベース補正のみ適用後)
    val absolutePitch: Float = 0f, // Pitch (X軸周りの絶対回転)
    val absoluteRoll: Float = 0f,  // Roll (Y軸周りの絶対回転)
    val absoluteYaw: Float = 0f,   // Yaw (Z軸周りの絶対回転)

    val isStationary: Boolean = false,
    val isHighSpeedMode: Boolean = false,
    val speedMps: Float = 0f,
    val speedKmh: Float = 0f,
    val totalDistance: Float = 0f,
    val accumulatedDistance: Float = 0f,
    val currentPoint: PointF? = null,
    val pathHistory: List<PointF> = emptyList(),
    val statusText: String = "初期化中",
    val statusColor: Int = android.R.color.darker_gray,
    val latestGpsData: LocationData? = null
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as MotionState
        if (!position.contentEquals(other.position)) return false
        if (!velocity.contentEquals(other.velocity)) return false
        if (!worldAccel.contentEquals(other.worldAccel)) return false
        if (relativeYaw != other.relativeYaw) return false
        if (relativePitch != other.relativePitch) return false
        if (relativeRoll != other.relativeRoll) return false
        if (absoluteYaw != other.absoluteYaw) return false
        if (absolutePitch != other.absolutePitch) return false
        if (absoluteRoll != other.absoluteRoll) return false
        if (isStationary != other.isStationary) return false
        if (isHighSpeedMode != other.isHighSpeedMode) return false
        if (speedMps != other.speedMps) return false
        if (speedKmh != other.speedKmh) return false
        if (totalDistance != other.totalDistance) return false
        if (accumulatedDistance != other.accumulatedDistance) return false
        if (currentPoint != other.currentPoint) return false
        if (pathHistory != other.pathHistory) return false
        if (statusText != other.statusText) return false
        if (statusColor != other.statusColor) return false
        if (latestGpsData != other.latestGpsData) return false
        return true
    }

    override fun hashCode(): Int {
        var result = position.contentHashCode()
        result = 31 * result + velocity.contentHashCode()
        result = 31 * result + worldAccel.contentHashCode()
        result = 31 * result + relativeYaw.hashCode()
        result = 31 * result + relativePitch.hashCode()
        result = 31 * result + relativeRoll.hashCode()
        result = 31 * result + absoluteYaw.hashCode()
        result = 31 * result + absolutePitch.hashCode()
        result = 31 * result + absoluteRoll.hashCode()
        result = 31 * result + isStationary.hashCode()
        result = 31 * result + isHighSpeedMode.hashCode()
        result = 31 * result + speedMps.hashCode()
        result = 31 * result + speedKmh.hashCode()
        result = 31 * result + totalDistance.hashCode()
        result = 31 * result + accumulatedDistance.hashCode()
        result = 31 * result + (currentPoint?.hashCode() ?: 0)
        result = 31 * result + pathHistory.hashCode()
        result = 31 * result + statusText.hashCode()
        result = 31 * result + statusColor
        result = 31 * result + (latestGpsData?.hashCode() ?: 0)
        return result
    }

    companion object {
        fun initial(): MotionState = MotionState()
    }
}