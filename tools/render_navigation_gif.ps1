param(
    [Parameter(Mandatory = $true)]
    [string]$Trace,
    [Parameter(Mandatory = $true)]
    [string]$Plan,
    [Parameter(Mandatory = $true)]
    [string]$MapPath,
    [Parameter(Mandatory = $true)]
    [string]$Output,
    [string]$Title = "Differential-drive A* navigation",
    [double]$MapContextM = 1.2,
    [int]$MaxFrames = 180,
    [int]$Fps = 12
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Drawing

$invariant = [Globalization.CultureInfo]::InvariantCulture

function Get-Number {
    param([object]$Value)
    return [double]::Parse([string]$Value, $invariant)
}

function Get-OptionalNumber {
    param([object]$Value)
    if ($null -eq $Value -or [string]::IsNullOrWhiteSpace([string]$Value)) {
        return $null
    }
    return Get-Number $Value
}

function Convert-Point {
    param(
        [double]$X,
        [double]$Y,
        [double]$XMin,
        [double]$XMax,
        [double]$YMin,
        [double]$YMax,
        [int]$Width,
        [int]$Height,
        [int]$Padding
    )
    $plotWidth = $Width - 2 * $Padding
    $plotHeight = $Height - 2 * $Padding
    $pixelX = $Padding + ($X - $XMin) / ($XMax - $XMin) * $plotWidth
    $pixelY = $Height - $Padding - ($Y - $YMin) / ($YMax - $YMin) * $plotHeight
    return [Drawing.PointF]::new([single]$pixelX, [single]$pixelY)
}

function Set-GifDelay {
    param([Drawing.Bitmap]$Bitmap, [int]$DelayHundredths)
    $property = [Runtime.Serialization.FormatterServices]::GetUninitializedObject(
        [Drawing.Imaging.PropertyItem]
    )
    $property.Id = 0x5100
    $property.Type = 4
    $property.Len = 4
    $property.Value = [BitConverter]::GetBytes([int]$DelayHundredths)
    $Bitmap.SetPropertyItem($property)
}

function Draw-Frame {
    param(
        [int]$FrameIndex,
        [object[]]$TraceRows,
        [object[]]$PlanRows,
        [object[]]$MapRows,
        [double]$XMin,
        [double]$XMax,
        [double]$YMin,
        [double]$YMax,
        [double]$GoalX,
        [double]$GoalY,
        [int]$Width,
        [int]$Height,
        [int]$Padding,
        [int]$Fps,
        [string]$Title
    )
    $bitmap = [Drawing.Bitmap]::new($Width, $Height)
    $graphics = [Drawing.Graphics]::FromImage($bitmap)
    $graphics.SmoothingMode = [Drawing.Drawing2D.SmoothingMode]::AntiAlias
    $graphics.TextRenderingHint = [Drawing.Text.TextRenderingHint]::AntiAliasGridFit
    $graphics.Clear([Drawing.Color]::White)

    $gridPen = [Drawing.Pen]::new([Drawing.Color]::FromArgb(225, 225, 225), 1)
    $mapBrush = [Drawing.SolidBrush]::new([Drawing.Color]::FromArgb(180, 82, 82, 82))
    $planPen = [Drawing.Pen]::new([Drawing.Color]::FromArgb(37, 99, 235), 2)
    $planPen.DashStyle = [Drawing.Drawing2D.DashStyle]::Dash
    $trajectoryPen = [Drawing.Pen]::new([Drawing.Color]::FromArgb(220, 38, 38), 3)
    $headingPen = [Drawing.Pen]::new([Drawing.Color]::FromArgb(15, 23, 42), 2)
    $startBrush = [Drawing.SolidBrush]::new([Drawing.Color]::FromArgb(22, 163, 74))
    $goalBrush = [Drawing.SolidBrush]::new([Drawing.Color]::FromArgb(245, 158, 11))
    $robotBrush = [Drawing.SolidBrush]::new([Drawing.Color]::FromArgb(14, 165, 233))
    $textBrush = [Drawing.SolidBrush]::new([Drawing.Color]::FromArgb(15, 23, 42))
    $font = [Drawing.Font]::new("Consolas", 10)
    $titleFont = [Drawing.Font]::new("Arial", 14, [Drawing.FontStyle]::Bold)

    for ($x = [Math]::Ceiling($XMin); $x -le $XMax; $x += 1.0) {
        $a = Convert-Point $x $YMin $XMin $XMax $YMin $YMax $Width $Height $Padding
        $b = Convert-Point $x $YMax $XMin $XMax $YMin $YMax $Width $Height $Padding
        $graphics.DrawLine($gridPen, $a, $b)
    }
    for ($y = [Math]::Ceiling($YMin); $y -le $YMax; $y += 1.0) {
        $a = Convert-Point $XMin $y $XMin $XMax $YMin $YMax $Width $Height $Padding
        $b = Convert-Point $XMax $y $XMin $XMax $YMin $YMax $Width $Height $Padding
        $graphics.DrawLine($gridPen, $a, $b)
    }

    $resolution = if ($MapRows.Count -gt 0) { Get-Number $MapRows[0].resolution_m } else { 0.1 }
    $cellA = Convert-Point $XMin $YMin $XMin $XMax $YMin $YMax $Width $Height $Padding
    $cellB = Convert-Point ($XMin + $resolution) ($YMin + $resolution) $XMin $XMax $YMin $YMax $Width $Height $Padding
    $cellWidth = [Math]::Max(1.0, [Math]::Abs($cellB.X - $cellA.X))
    $cellHeight = [Math]::Max(1.0, [Math]::Abs($cellB.Y - $cellA.Y))
    foreach ($mapRow in $MapRows) {
        $mapPoint = Convert-Point (Get-Number $mapRow.x_m) (Get-Number $mapRow.y_m) $XMin $XMax $YMin $YMax $Width $Height $Padding
        $rectangle = [Drawing.RectangleF]::new(
            [single]($mapPoint.X - $cellWidth / 2),
            [single]($mapPoint.Y - $cellHeight / 2),
            [single]$cellWidth,
            [single]$cellHeight
        )
        $graphics.FillRectangle($mapBrush, $rectangle)
    }

    $planPoints = New-Object Drawing.PointF[] $PlanRows.Count
    for ($i = 0; $i -lt $PlanRows.Count; $i++) {
        $planPoints[$i] = Convert-Point (Get-Number $PlanRows[$i].x_m) (Get-Number $PlanRows[$i].y_m) $XMin $XMax $YMin $YMax $Width $Height $Padding
    }
    if ($planPoints.Count -gt 1) {
        $graphics.DrawLines($planPen, $planPoints)
    }

    $current = $TraceRows[$FrameIndex]
    $trajectoryPoints = New-Object Drawing.PointF[] ($FrameIndex + 1)
    for ($i = 0; $i -le $FrameIndex; $i++) {
        $trajectoryPoints[$i] = Convert-Point (Get-Number $TraceRows[$i].x_m) (Get-Number $TraceRows[$i].y_m) $XMin $XMax $YMin $YMax $Width $Height $Padding
    }
    if ($trajectoryPoints.Count -gt 1) {
        $graphics.DrawLines($trajectoryPen, $trajectoryPoints)
    }

    $start = Convert-Point (Get-Number $TraceRows[0].x_m) (Get-Number $TraceRows[0].y_m) $XMin $XMax $YMin $YMax $Width $Height $Padding
    $goal = Convert-Point $GoalX $GoalY $XMin $XMax $YMin $YMax $Width $Height $Padding
    $robot = Convert-Point (Get-Number $current.x_m) (Get-Number $current.y_m) $XMin $XMax $YMin $YMax $Width $Height $Padding
    $graphics.FillEllipse($startBrush, $start.X - 6, $start.Y - 6, 12, 12)
    $graphics.FillEllipse($goalBrush, $goal.X - 9, $goal.Y - 9, 18, 18)
    $graphics.FillEllipse($robotBrush, $robot.X - 8, $robot.Y - 8, 16, 16)
    $yaw = Get-Number $current.yaw_rad
    $headingEnd = [Drawing.PointF]::new(
        [single]($robot.X + 25 * [Math]::Cos($yaw)),
        [single]($robot.Y - 25 * [Math]::Sin($yaw))
    )
    $graphics.DrawLine($headingPen, $robot, $headingEnd)

    $graphics.DrawString($Title, $titleFont, $textBrush, 18, 12)
    $clearance = Get-OptionalNumber $current.front_clearance_m
    $clearanceText = if ($null -eq $clearance) { "n/a" } else { "{0:F3} m" -f $clearance }
    $status = "t = {0,6:F1} s`nfront clearance = {1}`nsafety override = {2}" -f (Get-Number $current.time_s), $clearanceText, $current.safety_override
    $graphics.DrawString($status, $font, $textBrush, 20, 42)
    $graphics.DrawString("gray map   blue A* path   red /odom trajectory", $font, $textBrush, 20, $Height - 30)

    $gridPen.Dispose()
    $mapBrush.Dispose()
    $planPen.Dispose()
    $trajectoryPen.Dispose()
    $headingPen.Dispose()
    $startBrush.Dispose()
    $goalBrush.Dispose()
    $robotBrush.Dispose()
    $textBrush.Dispose()
    $font.Dispose()
    $titleFont.Dispose()
    $graphics.Dispose()
    Set-GifDelay $bitmap ([Math]::Max(1, [int][Math]::Round(100.0 / $Fps)))
    return $bitmap
}

$traceRows = @(Import-Csv -LiteralPath $Trace)
$planRows = @(Import-Csv -LiteralPath $Plan)
$mapRows = @(Import-Csv -LiteralPath $MapPath)
if ($traceRows.Count -eq 0 -or $planRows.Count -eq 0) {
    throw "Trace and plan must both contain rows."
}

$traceX = @($traceRows | ForEach-Object { Get-Number $_.x_m })
$traceY = @($traceRows | ForEach-Object { Get-Number $_.y_m })
$planX = @($planRows | ForEach-Object { Get-Number $_.x_m })
$planY = @($planRows | ForEach-Object { Get-Number $_.y_m })
$goalX = Get-OptionalNumber $traceRows[$traceRows.Count - 1].goal_x_m
$goalY = Get-OptionalNumber $traceRows[$traceRows.Count - 1].goal_y_m
if ($null -eq $goalX) { $goalX = $planX[$planX.Count - 1] }
if ($null -eq $goalY) { $goalY = $planY[$planY.Count - 1] }

$baseAllX = @($traceX + $planX + $goalX)
$baseAllY = @($traceY + $planY + $goalY)
$baseXMin = ([double]($baseAllX | Measure-Object -Minimum).Minimum) - 0.45
$baseXMax = ([double]($baseAllX | Measure-Object -Maximum).Maximum) + 0.45
$baseYMin = ([double]($baseAllY | Measure-Object -Minimum).Minimum) - 0.75
$baseYMax = ([double]($baseAllY | Measure-Object -Maximum).Maximum) + 0.75
$resolution = if ($mapRows.Count -gt 0) { Get-Number $mapRows[0].resolution_m } else { 0.1 }
$contextXMin = ([double]($baseAllX | Measure-Object -Minimum).Minimum) - $MapContextM
$contextXMax = ([double]($baseAllX | Measure-Object -Maximum).Maximum) + $MapContextM
$contextYMin = ([double]($baseAllY | Measure-Object -Minimum).Minimum) - $MapContextM
$contextYMax = ([double]($baseAllY | Measure-Object -Maximum).Maximum) + $MapContextM
$visibleMapRows = @($mapRows | Where-Object {
    $mapX = Get-Number $_.x_m
    $mapY = Get-Number $_.y_m
    $mapX -ge $contextXMin -and $mapX -le $contextXMax -and
        $mapY -ge $contextYMin -and $mapY -le $contextYMax
})
if ($visibleMapRows.Count -gt 0) {
    $mapMinX = ([double]($visibleMapRows | ForEach-Object { (Get-Number $_.x_m) - 0.5 * $resolution } | Measure-Object -Minimum).Minimum) - 0.15
    $mapMaxX = ([double]($visibleMapRows | ForEach-Object { (Get-Number $_.x_m) + 0.5 * $resolution } | Measure-Object -Maximum).Maximum) + 0.15
    $mapMinY = ([double]($visibleMapRows | ForEach-Object { (Get-Number $_.y_m) - 0.5 * $resolution } | Measure-Object -Minimum).Minimum) - 0.15
    $mapMaxY = ([double]($visibleMapRows | ForEach-Object { (Get-Number $_.y_m) + 0.5 * $resolution } | Measure-Object -Maximum).Maximum) + 0.15
    $xMin = [Math]::Min($baseXMin, $mapMinX)
    $xMax = [Math]::Max($baseXMax, $mapMaxX)
    $yMin = [Math]::Min($baseYMin, $mapMinY)
    $yMax = [Math]::Max($baseYMax, $mapMaxY)
} else {
    $xMin = $baseXMin
    $xMax = $baseXMax
    $yMin = $baseYMin
    $yMax = $baseYMax
}
$width = 960
$height = 600
$padding = 58
$frameCount = [Math]::Min([Math]::Max(2, $MaxFrames), $traceRows.Count)
$indices = @()
for ($i = 0; $i -lt $frameCount; $i++) {
    $indices += [int][Math]::Round($i * ($traceRows.Count - 1) / [double]($frameCount - 1))
}

$outputPath = [IO.Path]::GetFullPath($Output)
[IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName($outputPath)) | Out-Null
$gifCodec = [Drawing.Imaging.ImageCodecInfo]::GetImageEncoders() | Where-Object { $_.MimeType -eq "image/gif" }
$saveFlag = [Drawing.Imaging.Encoder]::SaveFlag
$encoderParams = [Drawing.Imaging.EncoderParameters]::new(1)
$encoderParams.Param[0] = [Drawing.Imaging.EncoderParameter]::new($saveFlag, [long][Drawing.Imaging.EncoderValue]::MultiFrame)
$frameParams = [Drawing.Imaging.EncoderParameters]::new(1)
$frameParams.Param[0] = [Drawing.Imaging.EncoderParameter]::new($saveFlag, [long][Drawing.Imaging.EncoderValue]::FrameDimensionTime)
$flushParams = [Drawing.Imaging.EncoderParameters]::new(1)
$flushParams.Param[0] = [Drawing.Imaging.EncoderParameter]::new($saveFlag, [long][Drawing.Imaging.EncoderValue]::Flush)

$first = $null
try {
    for ($frameNumber = 0; $frameNumber -lt $indices.Count; $frameNumber++) {
        $bitmap = Draw-Frame $indices[$frameNumber] $traceRows $planRows $visibleMapRows $xMin $xMax $yMin $yMax $goalX $goalY $width $height $padding $Fps $Title
        if ($frameNumber -eq 0) {
            $first = $bitmap
            $first.Save($outputPath, $gifCodec, $encoderParams)
        } else {
            $first.SaveAdd($bitmap, $frameParams)
            $bitmap.Dispose()
        }
    }
    $first.SaveAdd($flushParams)
} finally {
    if ($null -ne $first) { $first.Dispose() }
    $encoderParams.Dispose()
    $frameParams.Dispose()
    $flushParams.Dispose()
}

Write-Output "Wrote GIF to $outputPath"
