package com.example.test_gyro_1.filter

import android.util.Log
import com.example.test_gyro_1.gps.LocationData
import kotlin.math.max
import com.example.test_gyro_1.filter.SimpleMatrix // Assuming this is your Matrix class
import kotlin.math.sqrt

/**　メモ
 * 位置と速度を推定する拡張カルマンフィルタ
 * 状態ベクトル x = [px, py, pz, vx, vy, vz]^T (6x1)
 */
class ExtendedKalmanFilter {

    var x: SimpleMatrix //状態ベクトル [px, py, pz, vx, vy, vz] (6x1)
    var P: SimpleMatrix //状態推定誤差共分散行列 (6x6)

    private val accelNoiseStdDev = 0.8f // 加速度ノイズの標準偏差 (m/s^2)
    private val stationaryVelNoiseStdDev = 0.01f // 静止時速度ノイズの標準偏差 (m/s) bf0.01

    private var previousGpsDataUsedForUpdate: LocationData? = null

    companion object {
        private const val SHORT_DISTANCE_THRESHOLD = 1.0f // m (これ未満のGPS移動ではGPS更新を抑制) bf1.0

        // 通常のGPS観測ノイズ調整用パラメータ
        private const val GPS_XY_NOISE_FACTOR = 0.5f
        private const val GPS_Z_NOISE_FACTOR = 2.0f
        private const val GPS_MIN_STD_DEV = 1.0f
        private const val GPS_XY_NOISE_FACTOR_HIGH_SPEED = 0.1f // 高速時のXY信頼度を上げる (0.01は強すぎた可能性があるので少し戻す)

        // 強制GPS更新時のノイズ係数 (よりGPSを信頼する)　現状正式採用はちょい不明
        private const val GPS_XY_NOISE_FACTOR_FORCED = 0.05f // 通常の高速時よりさらに信頼
        private const val GPS_Z_NOISE_FACTOR_FORCED = 0.5f   // Z軸も通常より信頼
    }

    private val I = SimpleMatrix.identity(6)
    private val H_stationary = SimpleMatrix(3, 6) { r, c -> if (c == r + 3) 1.0f else 0.0f }
    private val R_stationary = SimpleMatrix(3, 3) { r, c ->
        if (r == c) stationaryVelNoiseStdDev * stationaryVelNoiseStdDev else 0.0f
    }

    private val F = SimpleMatrix.identity(6)

    private val H_gps_pos_3d = SimpleMatrix(3, 6) { r, c -> if (c == r) 1.0f else 0.0f }
    private val H_gps_pos_2d = SimpleMatrix(2, 6) { r, c -> if (c == r) 1.0f else 0.0f }


    init {
        reset() // 初期化処理をresetに集約
        Log.d("EKF", "EKF Initialized via constructor.")
    }

    /**　メモ
     * EKF 更新ステップ (GPS位置観測)
     * @param gpsData GPSマネージャーから受け取ったデータ
     * @param isHighSpeedMode 高速移動モードフラグ
     * @param isForcedUpdate 強制更新フラグ。trueの場合、短距離チェックをスキップし、専用ノイズ係数を使用。
     * @return Boolean: GPS更新が実際に実行されたかどうか (現状は簡易的にtrueを返すが、より厳密に判定しても良いかもね)
     */
    fun updateGpsPosition(gpsData: LocationData, isHighSpeedMode: Boolean, isForcedUpdate: Boolean = false): Boolean {
        if (!isForcedUpdate) { // 強制更新でない場合のみ短距離チェック
            val previousGps = previousGpsDataUsedForUpdate
            if (previousGps != null && gpsData.isLocalValid && previousGps.isLocalValid &&
                gpsData.localX != null && gpsData.localY != null &&
                previousGps.localX != null && previousGps.localY != null) {

                val dxGps = gpsData.localX - previousGps.localX
                val dyGps = gpsData.localY - previousGps.localY
                val gpsDistanceMoved = sqrt(dxGps * dxGps + dyGps * dyGps)

                if (gpsDistanceMoved < SHORT_DISTANCE_THRESHOLD) {
                    Log.d("EKF_Update_GPS", "Skipping GPS update: Short GPS distance (%.2fm < %.1fm)".format(gpsDistanceMoved, SHORT_DISTANCE_THRESHOLD))
                    return false // スキップ
                } else {
                    Log.d("EKF_Update_GPS", "Proceeding: GPS distance %.2fm >= %.1fm".format(gpsDistanceMoved, SHORT_DISTANCE_THRESHOLD))
                }
            } else {
                Log.d("EKF_Update_GPS", "Proceeding: First valid GPS or after reset.")
            }
        } else {
            Log.d("EKF_Update_GPS", "Proceeding with FORCED GPS update (short distance check bypassed).")
        }

        val maxAccuracy = 50.0f // 許容する最大GPS精度誤差　ここは現在確認中
        if (!gpsData.isLocalValid ||
            gpsData.localX == null || gpsData.localY == null ||
            gpsData.accuracy == null || gpsData.accuracy <= 0f || gpsData.accuracy > maxAccuracy) {
            val reason = when {
                !gpsData.isLocalValid -> "isLocalValid=false"
                gpsData.localX == null -> "localX=null"
                else -> "Unknown reason for invalid GPS"
            }
            Log.w("EKF_Update_GPS", "Skipping GPS update: Invalid GPS data ($reason). Accuracy: ${gpsData.accuracy}")
            return false
        }

        val gpsAccuracyStdDev = max(gpsData.accuracy, GPS_MIN_STD_DEV)
        val useZ = gpsData.localZ != null

        val H: SimpleMatrix
        val z: SimpleMatrix
        val R_gps: SimpleMatrix

        // ノイズ係数の選択
        val xyNoiseFactor: Float
        val zNoiseFactor: Float

        if (isForcedUpdate) {
            xyNoiseFactor = GPS_XY_NOISE_FACTOR_FORCED
            zNoiseFactor = GPS_Z_NOISE_FACTOR_FORCED
            Log.d("EKF_Update_GPS", "Using FORCED noise factors. XY: $xyNoiseFactor, Z: $zNoiseFactor")
        } else if (isHighSpeedMode) {
            xyNoiseFactor = GPS_XY_NOISE_FACTOR_HIGH_SPEED
            zNoiseFactor = GPS_Z_NOISE_FACTOR // 高速モードでもZの係数は通常と同じ（または専用のを定義しても良い）
            Log.d("EKF_Update_GPS", "Using HIGH_SPEED noise factors. XY: $xyNoiseFactor, Z: $zNoiseFactor")
        } else {
            xyNoiseFactor = GPS_XY_NOISE_FACTOR
            zNoiseFactor = GPS_Z_NOISE_FACTOR
            Log.d("EKF_Update_GPS", "Using NORMAL noise factors. XY: $xyNoiseFactor, Z: $zNoiseFactor")
        }

        if (useZ && gpsData.localZ != null) { // localZがnullでないことを再確認
            H = H_gps_pos_3d
            z = SimpleMatrix(3, 1); z[0,0]=gpsData.localX!!; z[1,0]=gpsData.localY!!; z[2,0]=gpsData.localZ!!

            val stdDevXY = gpsAccuracyStdDev * xyNoiseFactor
            val stdDevZ = gpsAccuracyStdDev * zNoiseFactor
            R_gps = SimpleMatrix(3, 3); R_gps[0,0]=stdDevXY*stdDevXY; R_gps[1,1]=stdDevXY*stdDevXY; R_gps[2,2]=stdDevZ*stdDevZ
            Log.d("EKF_Update_GPS", "3D GPS update. Acc: ${gpsData.accuracy}m, R_XY: %.4f, R_Z: %.4f".format(R_gps[0,0], R_gps[2,2]))
        } else {
            H = H_gps_pos_2d
            z = SimpleMatrix(2, 1); z[0,0]= gpsData.localX!!; z[1,0]= gpsData.localY!!

            val stdDevXY = gpsAccuracyStdDev * xyNoiseFactor
            R_gps = SimpleMatrix(2, 2); R_gps[0,0]=stdDevXY*stdDevXY; R_gps[1,1]=stdDevXY*stdDevXY
            Log.d("EKF_Update_GPS", "2D GPS update. Acc: ${gpsData.accuracy}m, R_XY: %.4f".format(R_gps[0,0]))
        }

        val y = z - (H * x)
        val P_HT = P * H.transpose()
        val S = H * P_HT + R_gps

        val S_inv: SimpleMatrix? = if (useZ && gpsData.localZ != null) S.inverse3x3() else S.inverse2x2()
        if (S_inv == null) {
            Log.e("EKF_Update_GPS", "Failed to invert S matrix. S:\n$S")
            return false
        }
        val K = P_HT * S_inv

        x += K * y
        P = (I - K * H) * P
        // P = (P + P.transpose()) * 0.5f // 対称性の保証

        Log.i("EKF_Update_GPS", "GPS Position Update Applied (${if(useZ && gpsData.localZ != null) "3D" else "2D"}). Forced: $isForcedUpdate")
        previousGpsDataUsedForUpdate = gpsData.copy() // 更新に使用したGPSデータを記録
        return true
    }

    fun reset() {
        x = SimpleMatrix(6, 1) // 位置(0,0,0), 速度(0,0,0)
        P = SimpleMatrix.identity(6) * 1.0f // 初期共分散 (ある程度の不確かさ) ここも要確認かなぁ

        // P[0,0]=P[1,1]=P[2,2] = 10.0f // 位置の初期不確かさを大きめに
        // P[3,3]=P[4,4]=P[5,5] = 1.0f  // 速度の初期不確かさ

        previousGpsDataUsedForUpdate = null
        Log.d("EKF", "EKF Reset. Initial State:\n$x \nInitial Covariance P:\n$P")
    }

    init { // プロパティ初期化後に呼ばれる
        x = SimpleMatrix(6, 1)
        P = SimpleMatrix.identity(6) * 1.0f
        reset() // 初期状態設定を reset に集約
    }

    fun predict(dt: Float, worldAccel: FloatArray) {
        if (dt <= 0) {
            Log.w("EKF_Predict", "dt is zero or negative ($dt), skipping prediction.")
            return
        }

        F[0, 3] = dt; F[1, 4] = dt; F[2, 5] = dt

        val dt2 = dt * dt
        val dt3_over_2 = 0.5f * dt * dt2
        val dt4_over_4 = 0.25f * dt2 * dt2
        val q_noise_variance = accelNoiseStdDev * accelNoiseStdDev

        // プロセスノイズ Q の構築 (dtに依存)
        val Q = SimpleMatrix(6, 6)
        Q[0, 0] = dt4_over_4 * q_noise_variance; Q[1, 1] = Q[0, 0]; Q[2, 2] = Q[0, 0]
        Q[3, 3] = dt2 * q_noise_variance;        Q[4, 4] = Q[3, 3]; Q[5, 5] = Q[3, 3]

        Q[0, 3] = dt3_over_2 * q_noise_variance; Q[3, 0] = Q[0, 3]
        Q[1, 4] = dt3_over_2 * q_noise_variance; Q[4, 1] = Q[1, 4]
        Q[2, 5] = dt3_over_2 * q_noise_variance; Q[5, 2] = Q[2, 5]
        // 他の非対角要素はゼロ（独立性を仮定している場合）

        val px = x[0, 0]; val py = x[1, 0]; val pz = x[2, 0]
        val vx = x[3, 0]; val vy = x[4, 0]; val vz = x[5, 0]
        val ax = worldAccel[0]; val ay = worldAccel[1]; val az = worldAccel[2]

        val x_pred = SimpleMatrix(6, 1)
        x_pred[0, 0] = px + vx * dt + 0.5f * ax * dt2
        x_pred[1, 0] = py + vy * dt + 0.5f * ay * dt2
        x_pred[2, 0] = pz + vz * dt + 0.5f * az * dt2
        x_pred[3, 0] = vx + ax * dt
        x_pred[4, 0] = vy + ay * dt
        x_pred[5, 0] = vz + az * dt

        val P_pred = F * P * F.transpose() + Q

        x = x_pred
        P = P_pred
    }

    fun updateStationary() {
        val z = SimpleMatrix(3, 1)
        val y = z - (H_stationary * x)
        val P_HT = P * H_stationary.transpose()
        val S = H_stationary * P_HT + R_stationary
        val S_inv = S.inverse3x3()
        if (S_inv == null) {
            Log.e("EKF_Stationary", "Failed to invert S matrix in stationary update. S:\n$S")
            // Pを少し大きくして発散を防ぐ試み (対角成分に微小値を加えるなど)
            // P.plusEquals(SimpleMatrix.identity(6).times(0.001f));
            return
        }
        val K = P_HT * S_inv
        x += K * y
        P = (I - K * H_stationary) * P
        // P = (P + P.transpose()) * 0.5f // 対称性の保証

        // 静止時は速度を明示的にゼロにする (より強く静止を反映)
        // x[3,0] = 0f; x[4,0] = 0f; x[5,0] = 0f;
        Log.d("EKF_Stationary", "Stationary Update Applied.")
    }

    fun getPosition(): FloatArray = floatArrayOf(x[0, 0], x[1, 0], x[2, 0])
    fun getVelocity(): FloatArray = floatArrayOf(x[3, 0], x[4, 0], x[5, 0])
}