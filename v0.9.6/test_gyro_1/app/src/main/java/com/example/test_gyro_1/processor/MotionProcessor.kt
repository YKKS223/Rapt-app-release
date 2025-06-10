package com.example.test_gyro_1.processor

import android.graphics.PointF
import android.hardware.SensorEvent
import android.hardware.SensorManager
import android.util.Log
import com.example.test_gyro_1.filter.ExtendedKalmanFilter
import com.example.test_gyro_1.gps.LocationData
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlin.math.pow
import kotlin.math.sqrt

class MotionProcessor {

    private val ekf = ExtendedKalmanFilter()
    private val quaternionProcessor = QuaternionProcessor()

    private var isUserOffsetSet = false
    private var hasRotationVectorData = false
    private var isMeasurementStarted = false

    private val worldAccel = FloatArray(3); private var accelLastTimestamp: Long = 0; private var isAccelFirstTime = true
    private var currentGyroRate = FloatArray(3); private var isStationary = false; private var isHighSpeedMode = false
    private var latestGpsLocationData: LocationData? = null; private var lastValidGpsData: LocationData? = null
    private var lastEkfPositionAtGpsUpdate: FloatArray? = null; private val positionHistory = mutableListOf<PointF>()
    private var lastPositionAddedToHistory: PointF? = null; private var accumulatedDistance: Float = 0f
    private var lastDistanceAccumulationTime: Long = 0L; private var lastPositionAtAccumulation: FloatArray? = null

    // 強制GNSS補正関連
    private var lastForcedGnssCorrectionTimeMs: Long = 0L

    companion object {
        // 定数
        private const val MIN_DISTANCE_TO_ADD_HISTORY = 0.1f // 履歴に追加する最小移動距離 (m)
        const val ACCUMULATION_INTERVAL_SECONDS = 1.0f // accumulatedDistance を計算する間隔 (秒) - 外部から参照・変更可能にする場合は public or internal bf1.0
        val ACCUMULATION_INTERVAL_MS = (ACCUMULATION_INTERVAL_SECONDS * 1000).toLong()
        private const val MIN_ACCUMULATION_DISTANCE_THRESHOLD = 0.05f // accumulatedDistance に加算する最小移動距離 (m)
        private const val MAX_ACCUMULATION_STEP_DISTANCE = 10.0f // accumulatedDistance に一度に加算できる最大移動距離 (m)
        private const val stationaryAccelThreshold = 0.2f // 静止判定の加速度閾値 (m/s^2)
        private const val stationaryGyroThreshold = 0.03f // 静止判定のジャイロ閾値 (rad/s)
        private const val MIN_VELOCITY_THRESHOLD = 0.1f // 速度がこれ未満の場合にゼロとみなす閾値 (m/s)
        private const val HIGH_SPEED_THRESHOLD_KMH = 5.0f // 高速モードと判定する速度閾値 (km/h)
        val HIGH_SPEED_THRESHOLD_MPS = (HIGH_SPEED_THRESHOLD_KMH / 3.6f) // m/s 単位
    }

    private val _motionStateFlow = MutableStateFlow(MotionState.initial())
    val motionStateFlow: StateFlow<MotionState> = _motionStateFlow.asStateFlow()

    fun reset() {
        Log.i("MotionProcessor", "Resetting MotionProcessor state...")
        ekf.reset(); quaternionProcessor.reset()
        isUserOffsetSet=false; hasRotationVectorData=false; isMeasurementStarted=false
        worldAccel.fill(0f); accelLastTimestamp=0L; isAccelFirstTime=true
        currentGyroRate.fill(0f); isStationary=false; isHighSpeedMode=false
        latestGpsLocationData=null; lastValidGpsData=null; lastEkfPositionAtGpsUpdate=null
        positionHistory.clear(); lastPositionAddedToHistory=null
        accumulatedDistance=0f; lastDistanceAccumulationTime=0L; lastPositionAtAccumulation=null
        lastForcedGnssCorrectionTimeMs = 0L
        _motionStateFlow.value = MotionState.initial()
        Log.i("MotionProcessor", "Processor reset complete.")
    }
    fun processGyroscopeEvent(event: SensorEvent) {
        currentGyroRate[0]=event.values[0]; currentGyroRate[1]=event.values[1]; currentGyroRate[2]=event.values[2]
    }
    fun processRotationVectorEvent(event: SensorEvent) {
        quaternionProcessor.processRotationVectorEvent(event)
        if (!hasRotationVectorData && quaternionProcessor.hasProcessedEvent()) {
            hasRotationVectorData=true; isMeasurementStarted=true
            Log.i("MotionProcessor", "Measurement started.")
        }
        if (isMeasurementStarted) updateMotionState()
    }
    fun calibrateCurrentOrientationAsZero() {
        if(quaternionProcessor.hasProcessedEvent()){
            val cq=quaternionProcessor.getCurrentCorrectedRawQuaternion()
            quaternionProcessor.setUserOffset(quaternionProcessor.invertQuaternion(cq))
            isUserOffsetSet=true; Log.i("MP","Offset calibrated.")
            updateMotionState()
        }else Log.w("MP","Can't calibrate: No rotation data.")
    }
    fun processLinearAccelerationEvent(event: SensorEvent) {
        if(!isMeasurementStarted)return
        val ts=event.timestamp
        if(isAccelFirstTime||accelLastTimestamp==0L){accelLastTimestamp=ts;isAccelFirstTime=false;return}
        val dt=(ts-accelLastTimestamp)*1e-9f
        if(dt<=1e-7f){accelLastTimestamp=ts;return} //非常に短いdtは無視
        accelLastTimestamp=ts

        val deviceToWorldRotationMatrix = quaternionProcessor.correctedRawRotationMatrix.takeIf { quaternionProcessor.hasProcessedEvent() }
        if (deviceToWorldRotationMatrix == null) {
            Log.w("MotionProcessor", "Raw rotation matrix not available for accel.")
            return
        }

        val dax=event.values[0];val day=event.values[1];val daz=event.values[2]
        worldAccel[0]=deviceToWorldRotationMatrix[0]*dax+deviceToWorldRotationMatrix[1]*day+deviceToWorldRotationMatrix[2]*daz
        worldAccel[1]=deviceToWorldRotationMatrix[3]*dax+deviceToWorldRotationMatrix[4]*day+deviceToWorldRotationMatrix[5]*daz
        worldAccel[2]=deviceToWorldRotationMatrix[6]*dax+deviceToWorldRotationMatrix[7]*day+deviceToWorldRotationMatrix[8]*daz

        // 静止状態判定
        val accelMagnitude = sqrt(worldAccel[0].pow(2)+worldAccel[1].pow(2)+worldAccel[2].pow(2))
        val gyroMagnitude = sqrt(currentGyroRate[0].pow(2)+currentGyroRate[1].pow(2)+currentGyroRate[2].pow(2))
        val gpsSpeedMagnitude = latestGpsLocationData?.speed ?: 0f
        val isGpsSpeedBelowThreshold = gpsSpeedMagnitude < HIGH_SPEED_THRESHOLD_MPS // ここでは高速モード閾値を使うが、静止判定用にもっと低い閾値でも良い

        val isCurrentlyImuStationary = accelMagnitude < stationaryAccelThreshold && gyroMagnitude < stationaryGyroThreshold
        val justBecameStationary = !isStationary && isCurrentlyImuStationary && isGpsSpeedBelowThreshold
        val remainStationary = isStationary && (accelMagnitude < stationaryAccelThreshold * 1.5f && gyroMagnitude < stationaryGyroThreshold * 1.5f) && isGpsSpeedBelowThreshold

        if(justBecameStationary || remainStationary){
            if(!isStationary) {
                isStationary = true
                Log.i("MotionProcessor", "Became stationary.")
            }
            ekf.updateStationary()
            ekf.predict(dt, floatArrayOf(0f,0f,0f)) // 静止時は加速度ゼロで予測
        } else {
            if(isStationary) {
                isStationary = false
                Log.i("MotionProcessor", "Became moving.")
            }
            ekf.predict(dt, worldAccel) // 通常予測
        }

        val ekfPositionBeforeGpsUpdate = ekf.getPosition().clone()

        // 定期的なGNSS強制補正 これはちょっと未定
        val currentTimeMs = System.currentTimeMillis()
        if (latestGpsLocationData != null && latestGpsLocationData!!.isLocalValid) {
            try {
                // isForcedUpdate = true で呼び出し
                ekf.updateGpsPosition(latestGpsLocationData!!, isHighSpeedMode, isForcedUpdate = true)
                lastValidGpsData = latestGpsLocationData!!.copy()
                lastEkfPositionAtGpsUpdate = ekf.getPosition().clone() // 強制更新後のEKF位置
            } catch (e: Exception) {
                Log.e("MotionProcessor", "EKF Forced GPS update error", e)
            }
            lastForcedGnssCorrectionTimeMs = currentTimeMs
        }

        // 通常のGPS更新
        val currentGpsDataToUse = latestGpsLocationData
        if (currentGpsDataToUse != null && currentGpsDataToUse.isLocalValid) {
            try {
                // isForcedUpdate = false (デフォルト) で呼び出し
                val updatedByNormalGps = ekf.updateGpsPosition(currentGpsDataToUse, isHighSpeedMode)
                if (updatedByNormalGps) { // EKF側で実際に更新が行われたかどうかのフラグを返す
                    lastValidGpsData = currentGpsDataToUse.copy()
                    lastEkfPositionAtGpsUpdate = ekfPositionBeforeGpsUpdate // 通常更新なら、更新前のEKF位置を記録
                }
            } catch (e: Exception) {
                Log.e("MotionProcessor", "EKF Normal GPS update error", e)
            }
        }

        // 低速度時の速度ゼロ化
        val currentEkfVelocity = ekf.getVelocity()
        val currentEkfSpeed = sqrt(currentEkfVelocity[0].pow(2) + currentEkfVelocity[1].pow(2) + currentEkfVelocity[2].pow(2))
        if(!isStationary && currentEkfSpeed < MIN_VELOCITY_THRESHOLD){
            // 速度が閾値未満なら、EKFの速度成分をゼロにする
            if(ekf.x[3,0] != 0f || ekf.x[4,0] != 0f || ekf.x[5,0] != 0f){
                ekf.x[3,0] = 0f; ekf.x[4,0] = 0f; ekf.x[5,0] = 0f
                Log.d("MotionProcessor", "Applied min velocity threshold. Speed was %.2f".format(currentEkfSpeed))
            }
        }

        // 累積距離の計算
        val currentEkfPosition = ekf.getPosition()
        val currentPositionPointF = PointF(currentEkfPosition[0], currentEkfPosition[1])
        val currentTimeForAccumulation = System.currentTimeMillis() // 累積計算用の現在時刻

        if (lastPositionAtAccumulation != null) {
            if (currentTimeForAccumulation - lastDistanceAccumulationTime >= ACCUMULATION_INTERVAL_MS) {
                val dx = currentEkfPosition[0] - lastPositionAtAccumulation!![0]
                val dy = currentEkfPosition[1] - lastPositionAtAccumulation!![1]
                val dz = currentEkfPosition[2] - lastPositionAtAccumulation!![2]
                val distDelta = sqrt(dx.pow(2) + dy.pow(2) + dz.pow(2))

                if (distDelta >= MIN_ACCUMULATION_DISTANCE_THRESHOLD && distDelta <= MAX_ACCUMULATION_STEP_DISTANCE) {
                    accumulatedDistance += distDelta
                }
                lastDistanceAccumulationTime = currentTimeForAccumulation
                lastPositionAtAccumulation = currentEkfPosition.clone()
            }
        } else {
            lastPositionAtAccumulation = currentEkfPosition.clone()
            lastDistanceAccumulationTime = currentTimeForAccumulation
        }

        // 軌跡履歴の追加
        var shouldAddToHistory = lastPositionAddedToHistory == null
        if (!shouldAddToHistory && lastPositionAddedToHistory != null) {
            val distFromLastHistoryPoint = sqrt((currentPositionPointF.x - lastPositionAddedToHistory!!.x).pow(2) + (currentPositionPointF.y - lastPositionAddedToHistory!!.y).pow(2))
            if (distFromLastHistoryPoint >= MIN_DISTANCE_TO_ADD_HISTORY) {
                shouldAddToHistory = true
            }
        }
        if (shouldAddToHistory) {
            positionHistory.add(currentPositionPointF)
            lastPositionAddedToHistory = currentPositionPointF
        }

        updateMotionState()
    }

    fun processGpsData(locationData: LocationData?) {
        this.latestGpsLocationData = locationData // 常に最新のGPSデータを保持
        if(locationData != null){
            val currentGpsSpeed = locationData.speed
            val isGpsValidForSpeedCheck = locationData.isLocalValid && (locationData.accuracy ?: Float.MAX_VALUE) < 30f // 精度30m以内

            if(currentGpsSpeed != null && isGpsValidForSpeedCheck){
                val enteredHighSpeed = currentGpsSpeed > HIGH_SPEED_THRESHOLD_MPS && !isHighSpeedMode
                val exitedHighSpeed = currentGpsSpeed <= HIGH_SPEED_THRESHOLD_MPS && isHighSpeedMode
                if(enteredHighSpeed){
                    isHighSpeedMode = true
                    Log.i("MotionProcessor","Entered High Speed Mode (GPS Speed: %.2fm/s)".format(currentGpsSpeed))
                } else if(exitedHighSpeed){
                    isHighSpeedMode = false
                    Log.i("MotionProcessor","Exited High Speed Mode (GPS Speed: %.2fm/s)".format(currentGpsSpeed))
                }
            } else { // GPS速度が無効またはGPS自体が無効な場合
                if(isHighSpeedMode){
                    isHighSpeedMode = false
                    Log.i("MotionProcessor","Exited High Speed Mode (Invalid GPS data for speed check)")
                }
            }
        } else { // locationData is null
            if(isHighSpeedMode){
                isHighSpeedMode = false
                Log.i("MotionProcessor","Exited High Speed Mode (GPS data is null)")
            }
        }
    }

    private val remappedRotationMatrix = FloatArray(9)

    private fun updateMotionState() {
        if (!quaternionProcessor.hasProcessedEvent()) {
            _motionStateFlow.value = MotionState.initial().copy(statusText = "センサー初期化中...")
            return
        }

        val currentEkfPosition = ekf.getPosition()
        val currentEkfVelocity = ekf.getVelocity()

        var userPitchAbs = 0f; var userRollAbs = 0f; var userYawAbs = 0f
        var userPitchRel = 0f; var userRollRel = 0f; var userYawRel = 0f

        val rawDeviceRotationMatrix = quaternionProcessor.correctedRawRotationMatrix.takeIf { quaternionProcessor.hasProcessedEvent() }
        if (rawDeviceRotationMatrix != null) {
            if (SensorManager.remapCoordinateSystem(rawDeviceRotationMatrix,
                    SensorManager.AXIS_X, SensorManager.AXIS_Z, remappedRotationMatrix)) {
                val remappedAnglesAbs = quaternionProcessor.getEulerAnglesFromRotationMatrix(remappedRotationMatrix)
                userYawAbs = -remappedAnglesAbs[0]
                userPitchAbs = remappedAnglesAbs[1]
                userRollAbs = -remappedAnglesAbs[2]
            } else { Log.w("MotionProcessor", "remapCoordinateSystem (Abs) failed.") }
        } else { Log.w("MotionProcessor", "rawDeviceRotationMatrix is null for Abs calculation.") }

        val finalDeviceRotationMatrix = quaternionProcessor.finalRotationMatrix.takeIf { quaternionProcessor.hasProcessedEvent() }
        if (finalDeviceRotationMatrix != null) {
            if (SensorManager.remapCoordinateSystem(finalDeviceRotationMatrix,
                    SensorManager.AXIS_X, SensorManager.AXIS_Z, remappedRotationMatrix)) {
                val remappedAnglesRel = quaternionProcessor.getEulerAnglesFromRotationMatrix(remappedRotationMatrix)
                userYawRel = -remappedAnglesRel[0] //　ここ変えない
                userPitchRel = -remappedAnglesRel[1] //　ここ変えない
                userRollRel = remappedAnglesRel[2]  //　ここ変えない　まじで
            } else { Log.w("MotionProcessor", "remapCoordinateSystem (Rel) failed.") }
        } else { Log.w("MotionProcessor", "finalDeviceRotationMatrix is null for Rel calculation.") }

        val statusText: String = if (isStationary) "状態: 静止中" else "状態: 移動中"
        val statusColor: Int = if (isStationary) android.R.color.holo_green_light else android.R.color.holo_orange_dark
        val ekfSpeedMps = sqrt(currentEkfVelocity[0].pow(2) + currentEkfVelocity[1].pow(2) + currentEkfVelocity[2].pow(2))

        val newState = MotionState(
            position = currentEkfPosition.clone(),
            velocity = currentEkfVelocity.clone(),
            worldAccel = worldAccel.clone(),
            relativePitch = userPitchRel, relativeRoll = userRollRel, relativeYaw = userYawRel,
            absolutePitch = userPitchAbs, absoluteRoll = userRollAbs, absoluteYaw = userYawAbs,
            isStationary = this.isStationary, isHighSpeedMode = this.isHighSpeedMode,
            speedMps = ekfSpeedMps,
            speedKmh = ekfSpeedMps * 3.6f,
            totalDistance = sqrt(currentEkfPosition[0].pow(2) + currentEkfPosition[1].pow(2) + currentEkfPosition[2].pow(2)),
            accumulatedDistance = this.accumulatedDistance,
            currentPoint = PointF(currentEkfPosition[0], currentEkfPosition[1]),
            pathHistory = positionHistory.toList(),
            statusText = statusText, statusColor = statusColor,
            latestGpsData = this.latestGpsLocationData?.copy()
        )
        if (_motionStateFlow.value != newState) {
            _motionStateFlow.value = newState
        }
    }
}