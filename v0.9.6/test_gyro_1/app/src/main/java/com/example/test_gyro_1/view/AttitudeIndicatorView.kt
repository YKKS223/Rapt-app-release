package com.example.test_gyro_1.view

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.util.AttributeSet
import android.util.Log
import android.view.View

class AttitudeIndicatorView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private var uiPitchValue: Float = 0f
    private var uiRollValue: Float = 0f
    private var uiYawValue: Float = 0f

    private val skyPaint = Paint().apply { color = Color.parseColor("#4A90E2"); style = Paint.Style.FILL; isAntiAlias = true }
    private val groundPaint = Paint().apply { color = Color.parseColor("#7ED321"); style = Paint.Style.FILL; isAntiAlias = true }
    private val horizonLinePaint = Paint().apply { color = Color.WHITE; strokeWidth = 4f; style = Paint.Style.STROKE; isAntiAlias = true }
    private val aircraftSymbolPaint = Paint().apply { color = Color.YELLOW; strokeWidth = 6f; style = Paint.Style.STROKE; strokeCap = Paint.Cap.ROUND; isAntiAlias = true }
    private val degreeMarkPaint = Paint().apply { color = Color.WHITE; strokeWidth = 2f; textSize = 20f; textAlign = Paint.Align.CENTER; isAntiAlias = true }

    private var viewCenterX: Float = 0f; private var viewCenterY: Float = 0f
    private var currentViewWidth: Int = 0; private var currentViewHeight: Int = 0

    private val aircraftPath = Path()
    private val pitchScaleFactor = 8f
    private val aspectRatio = 2.0f

    fun updateAttitude(pitchAngle: Float, yawAngle: Float, rollAngle: Float) {
        this.uiPitchValue = pitchAngle; this.uiYawValue = yawAngle; this.uiRollValue = rollAngle
        if (currentViewWidth > 0 && currentViewHeight > 0) invalidate()
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        currentViewWidth = w; currentViewHeight = h
        viewCenterX = w / 2f; viewCenterY = h / 2f
        aircraftPath.reset()
        val symbolBaseSize = h * 0.3f
        val pathSymbolWidth = symbolBaseSize; val pathSymbolWingLength = symbolBaseSize * 0.6f
        val pathSymbolWingAngleOffset = symbolBaseSize * 0.35f
        aircraftPath.moveTo(-pathSymbolWidth / 2, 0f); aircraftPath.lineTo(pathSymbolWidth / 2, 0f)
        aircraftPath.moveTo(-pathSymbolWidth / 2, 0f); aircraftPath.lineTo(-pathSymbolWidth / 2 - pathSymbolWingLength, pathSymbolWingAngleOffset)
        aircraftPath.moveTo(pathSymbolWidth / 2, 0f); aircraftPath.lineTo(pathSymbolWidth / 2 + pathSymbolWingLength, pathSymbolWingAngleOffset)
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (currentViewWidth == 0 || currentViewHeight == 0) return
        canvas.save(); canvas.translate(viewCenterX, viewCenterY)
        canvas.rotate(uiRollValue)
        val pitchOffsetPixels = uiPitchValue * pitchScaleFactor
        val drawWidthExtent = currentViewWidth * 2.0f; val drawHeightExtent = currentViewHeight * 2.0f
        canvas.drawRect(-drawWidthExtent, -drawHeightExtent - pitchOffsetPixels, drawWidthExtent, 0f - pitchOffsetPixels, skyPaint)
        canvas.drawRect(-drawWidthExtent, 0f - pitchOffsetPixels, drawWidthExtent, drawHeightExtent - pitchOffsetPixels, groundPaint)
        canvas.drawLine(-drawWidthExtent, 0f - pitchOffsetPixels, drawWidthExtent, 0f - pitchOffsetPixels, horizonLinePaint)
        drawPitchMarksRevisited(canvas, pitchOffsetPixels)
        canvas.restore()
        canvas.save(); canvas.translate(viewCenterX, viewCenterY)
        canvas.drawPath(aircraftPath, aircraftSymbolPaint)
        canvas.restore()
    }

    private fun drawPitchMarksRevisited(canvas: Canvas, horizonYOffset: Float) {
        if (currentViewWidth == 0 || currentViewHeight == 0) return
        val textOffsetY = degreeMarkPaint.textSize / 3; val markBaseWidth = this.currentViewWidth * 0.3f
        for (angle in -90..90 step 10) {
            val markYRelativeToHorizon = -(angle * pitchScaleFactor)
            val actualMarkY = markYRelativeToHorizon - horizonYOffset
            if (actualMarkY > currentViewHeight * 0.8f || actualMarkY < -currentViewHeight * 0.8f) continue
            var currentMarkLength = markBaseWidth * 0.25f
            if (angle == 0) continue else if (angle % 30 == 0) currentMarkLength = markBaseWidth * 0.5f
            canvas.drawLine(-currentMarkLength / 2, actualMarkY, currentMarkLength / 2, actualMarkY, degreeMarkPaint)
            if (angle != 0 && angle % 30 == 0) {
                val angleText = kotlin.math.abs(angle).toString()
                canvas.drawText(angleText, currentMarkLength / 2 + 30f, actualMarkY + textOffsetY, degreeMarkPaint)
                canvas.drawText(angleText, -currentMarkLength / 2 - 30f, actualMarkY + textOffsetY, degreeMarkPaint)
            }
        }
    }
    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val specifiedWidth = MeasureSpec.getSize(widthMeasureSpec)
        var calculatedHeight = (specifiedWidth / aspectRatio).toInt()
        val specifiedHeight = MeasureSpec.getSize(heightMeasureSpec)
        val heightMode = MeasureSpec.getMode(heightMeasureSpec)
        if (heightMode == MeasureSpec.EXACTLY) calculatedHeight = specifiedHeight
        else if (heightMode == MeasureSpec.AT_MOST) calculatedHeight = kotlin.math.min(calculatedHeight, specifiedHeight)
        val minHeight = suggestedMinimumHeight.coerceAtLeast(100)
        calculatedHeight = calculatedHeight.coerceAtLeast(minHeight)
        currentViewWidth = specifiedWidth; currentViewHeight = calculatedHeight
        setMeasuredDimension(currentViewWidth, currentViewHeight)
    }
}