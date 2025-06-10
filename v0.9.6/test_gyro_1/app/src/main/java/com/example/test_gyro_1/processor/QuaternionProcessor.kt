package com.example.test_gyro_1.processor

import android.hardware.SensorEvent
import android.hardware.SensorManager
import android.util.Log
import kotlin.math.*

class QuaternionProcessor {

    private var rawDeviceQuaternion = floatArrayOf(0f, 0f, 0f, 1f)
    private val baseCorrectionQuaternion: FloatArray // Identity
    private var correctedRawQuaternion = floatArrayOf(0f, 0f, 0f, 1f) //　実質raw
    private var userOffsetQuaternion = floatArrayOf(0f, 0f, 0f, 1f)
    private val finalQuaternion = floatArrayOf(0f, 0f, 0f, 1f) // userOffset * correctedRaw

    // 回転行列 - これらを public val に変更
    val correctedRawRotationMatrix = FloatArray(9)
    val finalRotationMatrix = FloatArray(9)
    private var hasProcessedEventSuccessfully = false

    init {
        baseCorrectionQuaternion = floatArrayOf(0f, 0f, 0f, 1f)
        Log.d("QuaternionProcessor", "BaseCorrectionQuaternion set to Identity.")
    }

    fun processRotationVectorEvent(event: SensorEvent) {
        if (event.values.size < 3) {
            Log.w("QuaternionProcessor", "Rotation vector event too short: ${event.values.size}")
            hasProcessedEventSuccessfully = false; return
        }
        val tempQuaternion = FloatArray(4)
        if (event.values.size == 3) SensorManager.getQuaternionFromVector(tempQuaternion, event.values)
        else {
            tempQuaternion[0]=event.values[0]; tempQuaternion[1]=event.values[1]; tempQuaternion[2]=event.values[2]
            tempQuaternion[3]=if(event.values.size>=4)event.values[3]else calculateW(event.values)
        }
        val norm=sqrt(tempQuaternion[0].pow(2)+tempQuaternion[1].pow(2)+tempQuaternion[2].pow(2)+tempQuaternion[3].pow(2))
        if(norm>1e-6f){
            rawDeviceQuaternion[0]=tempQuaternion[0]/norm; rawDeviceQuaternion[1]=tempQuaternion[1]/norm
            rawDeviceQuaternion[2]=tempQuaternion[2]/norm; rawDeviceQuaternion[3]=tempQuaternion[3]/norm
        }else{ Log.e("QuaternionProcessor", "Invalid quaternion from sensor (norm too small)."); hasProcessedEventSuccessfully = false; return}

        multiplyQuaternions(baseCorrectionQuaternion, rawDeviceQuaternion, correctedRawQuaternion)
        getRotationMatrixFromQuaternion(correctedRawQuaternion, correctedRawRotationMatrix) // プロパティに直接代入

        multiplyQuaternions(userOffsetQuaternion, correctedRawQuaternion, finalQuaternion)
        getRotationMatrixFromQuaternion(finalQuaternion, finalRotationMatrix) // プロパティに直接代入

        hasProcessedEventSuccessfully = true
    }

    private fun calculateW(v:FloatArray):Float{val s=v[0].pow(2)+v[1].pow(2)+v[2].pow(2);return if(s>=1f)0f else sqrt(1f-s)}

    fun getEulerAnglesFromRotationMatrix(rotationMatrix: FloatArray): FloatArray {
        val anglesRad = FloatArray(3)
        val anglesDeg = FloatArray(3)
        SensorManager.getOrientation(rotationMatrix, anglesRad)
        anglesDeg[0] = normalizeAngle180(Math.toDegrees(anglesRad[0].toDouble()).toFloat())
        anglesDeg[1] = Math.toDegrees(anglesRad[1].toDouble()).toFloat()
        anglesDeg[2] = Math.toDegrees(anglesRad[2].toDouble()).toFloat()
        return anglesDeg
    }

    fun getCurrentCorrectedRawQuaternion():FloatArray=correctedRawQuaternion.clone()
    fun setUserOffset(q:FloatArray){if(q.size==4){userOffsetQuaternion=q.clone();Log.d("QP","User offset set")}else Log.w("QP","Invalid offset quat")}
    fun resetUserOffset(){userOffsetQuaternion=floatArrayOf(0f,0f,0f,1f);Log.d("QP","User offset reset")}
    fun hasProcessedEvent():Boolean = hasProcessedEventSuccessfully
    fun reset(){rawDeviceQuaternion.fill(0f);rawDeviceQuaternion[3]=1f;baseCorrectionQuaternion[3]=1f;baseCorrectionQuaternion[0]=0f;baseCorrectionQuaternion[1]=0f;baseCorrectionQuaternion[2]=0f;correctedRawQuaternion.fill(0f);correctedRawQuaternion[3]=1f;userOffsetQuaternion.fill(0f);userOffsetQuaternion[3]=1f;finalQuaternion.fill(0f);finalQuaternion[3]=1f;correctedRawRotationMatrix.fill(0f);finalRotationMatrix.fill(0f);hasProcessedEventSuccessfully=false;Log.d("QP","Reset")}
    private fun normalizeAngle180(angle:Float):Float{var n=angle%360f;if(n>180f)n-=360f else if(n<=-180f)n+=360f;return n}
    private fun getRotationMatrixFromQuaternion(q:FloatArray,m:FloatArray){val x=q[0];val y=q[1];val z=q[2];val w=q[3];val x2=x*x;val y2=y*y;val z2=z*z;val xy=x*y;val xz=x*z;val yz=y*z;val wx=w*x;val wy=w*y;val wz=w*z;m[0]=1f-2f*(y2+z2);m[1]=2f*(xy-wz);m[2]=2f*(xz+wy);m[3]=2f*(xy+wz);m[4]=1f-2f*(x2+z2);m[5]=2f*(yz-wx);m[6]=2f*(xz-wy);m[7]=2f*(yz+wx);m[8]=1f-2f*(x2+y2)}
    private fun multiplyQuaternions(qA:FloatArray,qB:FloatArray,r:FloatArray){val wA=qA[3];val xA=qA[0];val yA=qA[1];val zA=qA[2];val wB=qB[3];val xB=qB[0];val yB=qB[1];val zB=qB[2];r[0]=xA*wB+yA*zB-zA*yB+wA*xB;r[1]=-xA*zB+yA*wB+zA*xB+wA*yB;r[2]=xA*yB-yA*xB+zA*wB+wA*zB;r[3]=-xA*xB-yA*yB-zA*zB+wA*wB}
    fun invertQuaternion(q:FloatArray):FloatArray=floatArrayOf(-q[0],-q[1],-q[2],q[3])
}