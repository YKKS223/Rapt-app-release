package com.example.test_gyro_1

import android.content.Context
import android.content.Intent
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.util.Log
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.core.content.ContextCompat
import androidx.lifecycle.lifecycleScope
import com.example.test_gyro_1.gps.GpsManager
import com.example.test_gyro_1.processor.MotionProcessor
import com.example.test_gyro_1.processor.MotionState
import com.example.test_gyro_1.view.AttitudeIndicatorView
import com.example.test_gyro_1.view.PathView
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.flow.sample
import kotlinx.coroutines.launch

class MainActivity : AppCompatActivity(), SensorEventListener {

    private lateinit var sensorManager: SensorManager
    private var gyroscope: Sensor? = null
    private var linearAccelerometer: Sensor? = null
    private var rotationVectorSensor: Sensor? = null

    private lateinit var textViewInfo: TextView
    private lateinit var textViewPitch: TextView
    private lateinit var textViewRoll: TextView
    private lateinit var textViewYaw: TextView

    private lateinit var textViewWarning: TextView
    private lateinit var textViewVelocity: TextView
    private lateinit var textViewDistance: TextView
    private lateinit var textViewWorldAcc: TextView
    private lateinit var textViewVelocityh: TextView
    private lateinit var textViewAccumulatedDistance: TextView
    private lateinit var buttonStopMeasurement: Button
    private lateinit var buttonCalibrateOrientation: Button
    private lateinit var pathView: PathView
    private lateinit var attitudeIndicatorView: AttitudeIndicatorView

    private lateinit var gpsManager: GpsManager
    private lateinit var motionProcessor: MotionProcessor


    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        bindViews()
        motionProcessor = MotionProcessor()
        gpsManager = GpsManager(this)
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        findSensors()

        buttonStopMeasurement.setOnClickListener { stopMeasurementAndShowResult() }
        buttonCalibrateOrientation.setOnClickListener {
            motionProcessor.calibrateCurrentOrientationAsZero()
            Toast.makeText(this, "現在の向きを基準に設定しました", Toast.LENGTH_SHORT).show()
            Log.i("MainActivity", "Orientation calibrated/reset by button.")
        }

        lifecycleScope.launch {
            // ★★★ 修正点 3 ★★★
            // sampleオペレータを追加し、UI更新を約15fps(66msごと)に間引きます。
            // これにより、描画負荷が大幅に削減され、アプリの応答性が向上します。
            motionProcessor.motionStateFlow
                .sample(16) // 60fps
                .collectLatest { state ->
                    val uiPitchForIndicator = state.absolutePitch
                    val uiRollForIndicator = state.absoluteRoll
                    val uiYawForIndicator = state.absoluteYaw

                    attitudeIndicatorView.updateAttitude(
                        uiPitchForIndicator,
                        uiYawForIndicator,
                        uiRollForIndicator
                    )

                    pathView.updatePath(state.pathHistory, state.currentPoint)
                    updateUI(state)
                }
        }

        lifecycleScope.launch {
            gpsManager.locationDataFlow.collect { locationData ->
                motionProcessor.processGpsData(locationData)
            }
        }

        textViewWarning.text = "状態: 計測中"
        updateUI(MotionState.initial())
    }

    private fun bindViews() {
        textViewInfo = findViewById(R.id.textViewInfo)
        textViewPitch = findViewById(R.id.textViewPitch)
        textViewRoll = findViewById(R.id.textViewRoll)
        textViewYaw = findViewById(R.id.textViewYaw)
        textViewWarning = findViewById(R.id.textViewWarning)
        textViewVelocity = findViewById(R.id.textViewVelocity)
        textViewDistance = findViewById(R.id.textViewDistance)
        textViewWorldAcc = findViewById(R.id.textViewWorldAcc)
        textViewVelocityh = findViewById(R.id.textViewVelocityh)
        textViewAccumulatedDistance = findViewById(R.id.textViewAccumulatedDistance)
        buttonStopMeasurement = findViewById(R.id.buttonStopMeasurement)
        buttonCalibrateOrientation = findViewById(R.id.buttonCalibrateOrientation)
        pathView = findViewById(R.id.pathView)
        attitudeIndicatorView = findViewById(R.id.attitudeIndicatorView)
    }

    override fun onResume() {
        super.onResume()
        Log.d("MainActivity", "onResume: Starting sensors and GPS")
        registerSensorListeners()
        gpsManager.startLocationUpdates()
    }

    override fun onPause() {
        super.onPause()
        Log.d("MainActivity", "onPause: Stopping sensors and GPS")
        unregisterSensorListeners()
        gpsManager.stopLocationUpdates()
    }

    private fun findSensors() {
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        linearAccelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
        rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
            ?: sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)

        if (rotationVectorSensor == null) {
            Log.e("MainActivity", "Rotation Vector Sensor (or Game Rotation Vector) not found!")
            textViewInfo.text = "回転ベクトルセンサーが見つかりません。"
        } else {
            textViewInfo.text = "センサー: 正常"
        }
    }

    private fun stopMeasurementAndShowResult() {
        Log.i("MainActivity", "Measurement Stopped (Button Clicked)")
        unregisterSensorListeners(); gpsManager.stopLocationUpdates()
        val finalState = motionProcessor.motionStateFlow.value
        textViewWarning.text = "状態: 計測終了"; textViewWarning.setTextColor(ContextCompat.getColor(this, android.R.color.darker_gray))
        buttonStopMeasurement.isEnabled = false
        val pathX = finalState.pathHistory.map { it.x }.toFloatArray(); val pathY = finalState.pathHistory.map { it.y }.toFloatArray()
        val intent = Intent(this, ResultActivity::class.java).apply {
            putExtra(ResultActivity.EXTRA_ANGLE_X, finalState.relativePitch)
            putExtra(ResultActivity.EXTRA_ANGLE_Y, finalState.relativeRoll)
            putExtra(ResultActivity.EXTRA_ANGLE_Z, finalState.relativeYaw)
            putExtra(ResultActivity.EXTRA_DISPLACEMENT, finalState.totalDistance)
            putExtra(ResultActivity.EXTRA_TOTAL_DISTANCE, finalState.accumulatedDistance)
            putExtra(ResultActivity.EXTRA_PATH_X, pathX); putExtra(ResultActivity.EXTRA_PATH_Y, pathY)
        }
        finalState.currentPoint?.let { intent.putExtra(ResultActivity.EXTRA_CURRENT_POINT_X, it.x); intent.putExtra(ResultActivity.EXTRA_CURRENT_POINT_Y, it.y) }
        startActivity(intent)
    }

    private fun registerSensorListeners() {
        val sensorDelay = SensorManager.SENSOR_DELAY_GAME
        gyroscope?.let { sensorManager.registerListener(this, it, sensorDelay) }
        linearAccelerometer?.let { sensorManager.registerListener(this, it, sensorDelay) }
        rotationVectorSensor?.let { sensorManager.registerListener(this, it, sensorDelay) }
        Log.d("MainActivity", "Sensor listeners registered with delay: $sensorDelay")
    }

    private fun unregisterSensorListeners() {
        sensorManager.unregisterListener(this)
        Log.d("MainActivity", "Sensor listeners unregistered.")
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        Log.d("MainActivity", "Sensor ${sensor?.name} accuracy changed to $accuracy")
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (event == null) return
        // ★★★ 修正点 2 ★★★
        // センサー処理（行列計算など）をバックグラウンドスレッドに移行し、UIスレッドの負荷を軽減します。
        // これにより、UIが固まるのを防ぎます。
        lifecycleScope.launch(Dispatchers.Default) {
            when (event.sensor.type) {
                Sensor.TYPE_GYROSCOPE -> motionProcessor.processGyroscopeEvent(event)
                Sensor.TYPE_LINEAR_ACCELERATION -> motionProcessor.processLinearAccelerationEvent(event)
                Sensor.TYPE_ROTATION_VECTOR, Sensor.TYPE_GAME_ROTATION_VECTOR -> motionProcessor.processRotationVectorEvent(event)
            }
        }
    }

    private fun updateUI(state: MotionState) {
        runOnUiThread {
            textViewPitch.text = "Pitch (X)\nAbs: %.1f°\nRel: %.1f°".format(state.absolutePitch, state.relativePitch)
            textViewRoll.text = "Roll (Y)\nAbs: %.1f°\nRel: %.1f°".format(state.absoluteRoll, state.relativeRoll)
            textViewYaw.text = "Yaw (Z)\nAbs: %.1f°\nRel: %.1f°".format(state.absoluteYaw, state.relativeYaw)
            textViewWorldAcc.text = "加速(W): X:%.2f Y:%.2f Z:%.2f".format(state.worldAccel[0], state.worldAccel[1], state.worldAccel[2])
            textViewVelocity.text = "速度: %.2f m/s".format(state.speedMps); textViewVelocityh.text = "時速: %.1f km/h".format(state.speedKmh)
            textViewDistance.text = "変位(直): %.2f m".format(state.totalDistance); textViewAccumulatedDistance.text = "総移動距離: %.2f m".format(state.accumulatedDistance)
            textViewWarning.text = state.statusText
            try { textViewWarning.setTextColor(ContextCompat.getColor(this, state.statusColor)) }
            catch (e: Exception) { Log.e("MainActivity", "Failed to set text color", e); textViewWarning.setTextColor(ContextCompat.getColor(this, android.R.color.darker_gray)) }
        }
    }
}