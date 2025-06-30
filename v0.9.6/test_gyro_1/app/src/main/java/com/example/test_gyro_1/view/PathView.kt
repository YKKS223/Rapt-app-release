package com.example.test_gyro_1.view

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.PointF
import android.util.AttributeSet
import android.util.Log
import android.view.View
import kotlin.math.max
import kotlin.math.min

class PathView @JvmOverloads constructor(
    context: Context, attrs: AttributeSet? = null, defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private var currentPoint: PointF? = null
    private val path = Path()
    private val paint = Paint().apply {
        color = Color.BLUE
        strokeWidth = 5f
        style = Paint.Style.STROKE
        isAntiAlias = true
    }
    private val currentPointPaint = Paint().apply {
        color = Color.GREEN
        style = Paint.Style.FILL
        isAntiAlias = true
    }
    private val originPointPaint = Paint().apply {
        color = Color.RED
        style = Paint.Style.FILL
        isAntiAlias = true
    }

    private var minX = 0f
    private var maxX = 0f
    private var minY = 0f
    private var maxY = 0f
    private var scale = 5f
    private var offsetX = 0f
    private var offsetY = 0f

    private val aspectRatio = 2.0f // 横2 : 縦1

    // 最後に描画したポイントのリストを保持 (onSizeChangedでの再計算用)
    private var lastPointsForBounds: List<PointF> = emptyList()

    fun updatePath(newPoints: List<PointF>, currentPoint: PointF?) {
        this.lastPointsForBounds = newPoints + listOfNotNull(currentPoint) // 現在位置も含めて保持
        this.currentPoint = currentPoint

        if (width > 0 && height > 0) { // サイズが確定していれば境界を計算
            calculateBoundsAndScale(this.lastPointsForBounds)
        }

        path.rewind()
        if (newPoints.isNotEmpty()) {
            if (scale > 0) { // スケールが計算済みの場合のみパス構築
                val start = transformPoint(newPoints[0])
                path.moveTo(start.x, start.y)
                for (i in 1 until newPoints.size) {
                    val point = transformPoint(newPoints[i])
                    path.lineTo(point.x, point.y)
                }
            }
        }
        invalidate()
    }

    private fun calculateBoundsAndScale(points: List<PointF>) {
        if (points.isEmpty() || width == 0 || height == 0) {
            // Log.d("PathView", "calculateBoundsAndScale skipped: no points or zero dimensions.")
            this.scale = 5f // デフォルトに戻すなど
            this.offsetX = width / 2f
            this.offsetY = height / 2f
            return
        }

        minX = points.minOf { it.x }
        maxX = points.maxOf { it.x }
        minY = points.minOf { it.y }
        maxY = points.maxOf { it.y }

        val viewWidth = width.toFloat()
        val viewHeight = height.toFloat()

        val requiredWidthMeters = max(0.1f, maxX - minX) // 0除算を避けるため最小値を設定
        val requiredHeightMeters = max(0.1f, maxY - minY)

        val padding = 30f * 2
        val availableWidth = viewWidth - padding
        val availableHeight = viewHeight - padding

        if (availableWidth <= 0 || availableHeight <= 0) {
            // Log.d("PathView", "calculateBoundsAndScale skipped: no available draw area after padding.")
            this.scale = 1f // 描画領域がない場合は最小スケール
            this.offsetX = viewWidth / 2f
            this.offsetY = viewHeight / 2f
            return
        }

        val scaleX = availableWidth / requiredWidthMeters
        val scaleY = availableHeight / requiredHeightMeters

        scale = min(scaleX, scaleY)
        if (scale <= 0.01f) scale = 0.01f // スケールが極小または0以下になるのを防ぐ

        val pathCenterX = (minX + maxX) / 2f
        val pathCenterY = (minY + maxY) / 2f
        offsetX = viewWidth / 2f - pathCenterX * scale
        offsetY = viewHeight / 2f + pathCenterY * scale // Y軸反転を考慮
        // Log.d("PathView", "Calculated: scale=$scale, offsetX=$offsetX, offsetY=$offsetY")
    }

    private fun transformPoint(point: PointF): PointF {
        val viewX = point.x * scale + offsetX
        val viewY = -point.y * scale + offsetY
        return PointF(viewX, viewY)
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (width == 0 || height == 0) return

        // ★★★ 修正点 1 (最重要) ★★★
        // onDraw内で毎回スケール計算をしていた重い処理を削除します。
        // これがフリーズの主な原因でした。
        // スケールの再計算は updatePath と onSizeChanged でのみ行われます。
        /*
        if(this.lastPointsForBounds.isNotEmpty()){
            calculateBoundsAndScale(this.lastPointsForBounds)
        }
        */

        canvas.drawPath(path, paint)

        if (scale > 0) { // スケールが計算されている場合のみ描画
            val origin = transformPoint(PointF(0f, 0f))
            canvas.drawCircle(origin.x, origin.y, 10f, originPointPaint)
            currentPoint?.let {
                val currentViewPoint = transformPoint(it)
                canvas.drawCircle(currentViewPoint.x, currentViewPoint.y, 8f, currentPointPaint)
            }
        }
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        // サイズ変更時に保存されている最新のポイントで境界とスケールを再計算
        if (this.lastPointsForBounds.isNotEmpty()) {
            calculateBoundsAndScale(this.lastPointsForBounds)
        }
        invalidate() // 再描画を強制
    }

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        super.onMeasure(widthMeasureSpec, heightMeasureSpec)

        val widthMode = MeasureSpec.getMode(widthMeasureSpec)
        val currentWidth = MeasureSpec.getSize(widthMeasureSpec)

        var finalWidth = currentWidth
        var finalHeight = (currentWidth / aspectRatio).toInt()

        if (widthMode == MeasureSpec.EXACTLY) {
            finalWidth = currentWidth
            finalHeight = (currentWidth / aspectRatio).toInt()
        } else if (widthMode == MeasureSpec.AT_MOST) {
            finalWidth = currentWidth
            finalHeight = (currentWidth / aspectRatio).toInt()
        } else {
            finalWidth = measuredWidth
            finalHeight = (finalWidth / aspectRatio).toInt()
        }

        if (finalHeight <= 0 && finalWidth > 0) {
            finalHeight = (finalWidth / aspectRatio).toInt().coerceAtLeast(1)
        }
        if (finalWidth <= 0 && finalHeight > 0) {
            finalWidth = (finalHeight * aspectRatio).toInt().coerceAtLeast(1)
        }

        setMeasuredDimension(finalWidth, finalHeight)
        Log.d("PathView", "onMeasure: finalWidth=$finalWidth, finalHeight=$finalHeight, currentWidth=$currentWidth")
    }
}