param(
    [string]$Port = "COM3",
    [int]$BaudRate = 115200,
    [ValidateRange(1, 100)]
    [int]$ExposureMs = 40,
    [ValidateRange(10, 90)]
    [int]$ThresholdPercent = 45,
    [ValidateRange(50, 2000)]
    [int]$IntervalMs = 150,
    [switch]$Once
)

$ErrorActionPreference = "Stop"

$script:serial = $null
$script:frameNumber = 0

function Open-CcdSerial {
    if (($null -ne $script:serial) -and $script:serial.IsOpen) {
        return
    }

    $script:serial = [System.IO.Ports.SerialPort]::new(
        $Port, $BaudRate, [System.IO.Ports.Parity]::None, 8,
        [System.IO.Ports.StopBits]::One)
    $script:serial.ReadTimeout = 100
    $script:serial.WriteTimeout = 1000
    $script:serial.NewLine = "`r`n"
    $script:serial.DtrEnable = $false
    $script:serial.RtsEnable = $false
    $script:serial.Open()
}

function Close-CcdSerial {
    if ($null -ne $script:serial) {
        if ($script:serial.IsOpen) {
            $script:serial.Close()
        }
        $script:serial.Dispose()
        $script:serial = $null
    }
}

function Read-CcdFrame {
    param(
        [int]$FrameExposureMs,
        [int]$FrameThresholdPercent
    )

    Open-CcdSerial
    $pixels = [int[]]::new(128)
    $received = [bool[]]::new(128)
    $peakCenters = [System.Collections.Generic.List[int]]::new()
    $peakWidths = [System.Collections.Generic.List[int]]::new()
    $minimum = -1
    $maximum = -1
    $span = -1
    $threshold = -1
    $reportedPeakCount = -1
    $complete = $false

    $script:serial.DiscardInBuffer()
    $script:serial.WriteLine(
        "ccd dump $FrameExposureMs $FrameThresholdPercent")

    $timeoutMs = [Math]::Max(1800, ($FrameExposureMs * 5) + 1000)
    $deadline = [DateTime]::UtcNow.AddMilliseconds($timeoutMs)
    while ([DateTime]::UtcNow -lt $deadline) {
        try {
            $line = $script:serial.ReadLine().Trim()
        }
        catch [System.TimeoutException] {
            continue
        }

        if ($line -match
            "^CCD_STATS min=(\d+) max=(\d+) span=(\d+) threshold=(\d+) peaks=(\d+)") {
            $minimum = [int]$Matches[1]
            $maximum = [int]$Matches[2]
            $span = [int]$Matches[3]
            $threshold = [int]$Matches[4]
            $reportedPeakCount = [int]$Matches[5]
            continue
        }

        if ($line -match "^CCD_PEAKS(?:\s+(.*))?$") {
            $peakCenters.Clear()
            $peakWidths.Clear()
            $peakText = $Matches[1]
            if (-not [string]::IsNullOrWhiteSpace($peakText)) {
                foreach ($token in ($peakText.Trim() -split "\s+")) {
                    if ($token -match "^(\d+):(\d+)$") {
                        $peakCenters.Add([int]$Matches[1])
                        $peakWidths.Add([int]$Matches[2])
                    }
                }
            }
            continue
        }

        if ($line -match "^CCD_DATA\s+(\d{3})\s+(.+)$") {
            $start = [int]$Matches[1]
            $values = $Matches[2].Trim() -split "\s+"
            for ($offset = 0; $offset -lt $values.Count; $offset++) {
                $index = $start + $offset
                if ($index -ge 128) {
                    break
                }
                $value = 0
                if ([int]::TryParse(
                        $values[$offset],
                        [ref]$value)) {
                    $pixels[$index] = $value
                    $received[$index] = $true
                }
            }
            continue
        }

        if ($line -eq "OK ccd capture complete; ADC0 restored to PA27") {
            $complete = $true
            break
        }
        if ($line.StartsWith("ERR ")) {
            throw $line
        }
    }

    if (-not $complete) {
        throw "Timed out waiting for a complete CCD frame."
    }
    $receivedCount = ($received | Where-Object { $_ }).Count
    if ($receivedCount -ne 128) {
        throw "Incomplete CCD frame: received $receivedCount of 128 pixels."
    }

    if (($minimum -lt 0) -or ($maximum -lt 0)) {
        $minimum = ($pixels | Measure-Object -Minimum).Minimum
        $maximum = ($pixels | Measure-Object -Maximum).Maximum
        $span = $maximum - $minimum
    }

    $brightestValue = -1
    $brightestPixel = 0
    for ($index = 0; $index -lt 128; $index++) {
        if ($pixels[$index] -gt $brightestValue) {
            $brightestValue = $pixels[$index]
            $brightestPixel = $index
        }
    }

    $script:frameNumber++
    return [pscustomobject]@{
        Number = $script:frameNumber
        Time = [DateTime]::Now
        Pixels = $pixels
        Minimum = $minimum
        Maximum = $maximum
        Span = $span
        Threshold = $threshold
        PeakCount = $reportedPeakCount
        PeakCenters = $peakCenters.ToArray()
        PeakWidths = $peakWidths.ToArray()
        BrightestPixel = $brightestPixel
        BrightestValue = $brightestValue
        ExposureMs = $FrameExposureMs
        ThresholdPercent = $FrameThresholdPercent
    }
}

if ($Once) {
    try {
        $frame = Read-CcdFrame $ExposureMs $ThresholdPercent
        Write-Output (
            ("CCD frame={0} min={1} max={2} span={3} peaks={4} " +
             "brightest={5}:{6}") -f
            $frame.Number, $frame.Minimum, $frame.Maximum, $frame.Span,
            $frame.PeakCount, $frame.BrightestPixel, $frame.BrightestValue)
        Write-Output ("CCD pixels=" + ($frame.Pixels -join ","))
    }
    finally {
        Close-CcdSerial
    }
    exit
}

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

[System.Windows.Forms.Application]::EnableVisualStyles()

$form = [System.Windows.Forms.Form]::new()
$form.Text = "TSL1401 Live Pixel View"
$form.StartPosition = "CenterScreen"
$form.ClientSize = [System.Drawing.Size]::new(1120, 700)
$form.MinimumSize = [System.Drawing.Size]::new(800, 500)
$form.KeyPreview = $true

$toolbar = [System.Windows.Forms.FlowLayoutPanel]::new()
$toolbar.Dock = "Top"
$toolbar.Height = 48
$toolbar.Padding = [System.Windows.Forms.Padding]::new(8, 8, 8, 4)
$toolbar.WrapContents = $false

$portLabel = [System.Windows.Forms.Label]::new()
$portLabel.Text = "$Port  $BaudRate baud"
$portLabel.AutoSize = $true
$portLabel.Margin = [System.Windows.Forms.Padding]::new(4, 7, 18, 0)

$exposureLabel = [System.Windows.Forms.Label]::new()
$exposureLabel.Text = "Exposure ms"
$exposureLabel.AutoSize = $true
$exposureLabel.Margin = [System.Windows.Forms.Padding]::new(4, 7, 4, 0)

$exposureControl = [System.Windows.Forms.NumericUpDown]::new()
$exposureControl.Minimum = 1
$exposureControl.Maximum = 100
$exposureControl.Value = $ExposureMs
$exposureControl.Width = 62

$thresholdLabel = [System.Windows.Forms.Label]::new()
$thresholdLabel.Text = "Threshold %"
$thresholdLabel.AutoSize = $true
$thresholdLabel.Margin = [System.Windows.Forms.Padding]::new(14, 7, 4, 0)

$thresholdControl = [System.Windows.Forms.NumericUpDown]::new()
$thresholdControl.Minimum = 10
$thresholdControl.Maximum = 90
$thresholdControl.Value = $ThresholdPercent
$thresholdControl.Width = 62

$toggleButton = [System.Windows.Forms.Button]::new()
$toggleButton.Text = "Pause"
$toggleButton.AutoSize = $true
$toggleButton.Margin = [System.Windows.Forms.Padding]::new(18, 0, 4, 0)

$toolbar.Controls.AddRange(@(
    $portLabel, $exposureLabel, $exposureControl,
    $thresholdLabel, $thresholdControl, $toggleButton))

$statusLabel = [System.Windows.Forms.Label]::new()
$statusLabel.Dock = "Top"
$statusLabel.Height = 34
$statusLabel.Padding = [System.Windows.Forms.Padding]::new(12, 7, 8, 4)
$statusLabel.Font = [System.Drawing.Font]::new(
    "Consolas", 10, [System.Drawing.FontStyle]::Regular)
$statusLabel.Text = "Connecting to $Port..."

$hintLabel = [System.Windows.Forms.Label]::new()
$hintLabel.Dock = "Bottom"
$hintLabel.Height = 30
$hintLabel.Padding = [System.Windows.Forms.Padding]::new(12, 6, 8, 4)
$hintLabel.Text =
    "Adjust the sensor so the intended screen center is near pixel 63.5. " +
    "Avoid max values near 4095 and keep span comfortably above 100."

$plotPanel = [System.Windows.Forms.Panel]::new()
$plotPanel.Dock = "Fill"
$plotPanel.BackColor = [System.Drawing.Color]::FromArgb(20, 22, 25)
$doubleBufferedProperty = $plotPanel.GetType().GetProperty(
    "DoubleBuffered",
    [System.Reflection.BindingFlags]::Instance -bor
    [System.Reflection.BindingFlags]::NonPublic)
$doubleBufferedProperty.SetValue($plotPanel, $true, $null)

$script:latestFrame = $null
$script:running = $true

$plotPanel.Add_Paint({
    param($sender, $eventArgs)

    $graphics = $eventArgs.Graphics
    $graphics.SmoothingMode =
        [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
    $width = $sender.ClientSize.Width
    $height = $sender.ClientSize.Height
    $left = 66
    $right = 24
    $top = 22
    $bottom = 48
    $plotWidth = [Math]::Max(1, $width - $left - $right)
    $plotHeight = [Math]::Max(1, $height - $top - $bottom)

    $gridPen = [System.Drawing.Pen]::new(
        [System.Drawing.Color]::FromArgb(58, 64, 70), 1)
    $axisPen = [System.Drawing.Pen]::new(
        [System.Drawing.Color]::FromArgb(145, 150, 155), 1)
    $centerPen = [System.Drawing.Pen]::new(
        [System.Drawing.Color]::FromArgb(0, 200, 220), 2)
    $centerPen.DashStyle = [System.Drawing.Drawing2D.DashStyle]::Dash
    $tracePen = [System.Drawing.Pen]::new(
        [System.Drawing.Color]::FromArgb(245, 220, 45), 2)
    $peakPen = [System.Drawing.Pen]::new(
        [System.Drawing.Color]::FromArgb(255, 80, 80), 2)
    $labelBrush = [System.Drawing.SolidBrush]::new(
        [System.Drawing.Color]::FromArgb(210, 215, 220))
    $mutedBrush = [System.Drawing.SolidBrush]::new(
        [System.Drawing.Color]::FromArgb(145, 150, 155))
    $font = [System.Drawing.Font]::new("Consolas", 9)

    try {
        for ($value = 0; $value -le 4096; $value += 512) {
            $y = $top + $plotHeight -
                (($value / 4095.0) * $plotHeight)
            $graphics.DrawLine(
                $gridPen, $left, [float]$y,
                $left + $plotWidth, [float]$y)
            if (($value % 1024) -eq 0) {
                $label = [Math]::Min($value, 4095).ToString()
                $graphics.DrawString(
                    $label, $font, $mutedBrush, 8, [float]($y - 8))
            }
        }

        foreach ($pixel in @(0, 16, 32, 48, 64, 80, 96, 112, 127)) {
            $x = $left + (($pixel / 127.0) * $plotWidth)
            $graphics.DrawLine(
                $gridPen, [float]$x, $top,
                [float]$x, $top + $plotHeight)
            $graphics.DrawString(
                $pixel.ToString(), $font, $mutedBrush,
                [float]($x - 10), $top + $plotHeight + 9)
        }

        $graphics.DrawRectangle(
            $axisPen, $left, $top, $plotWidth, $plotHeight)
        $centerX = $left + ((63.5 / 127.0) * $plotWidth)
        $graphics.DrawLine(
            $centerPen, [float]$centerX, $top,
            [float]$centerX, $top + $plotHeight)
        $graphics.DrawString(
            "center 63.5", $font, $labelBrush,
            [float]($centerX + 5), [float]($top + 4))

        if ($null -eq $script:latestFrame) {
            $graphics.DrawString(
                "Waiting for the first 128-pixel frame...",
                [System.Drawing.Font]::new("Segoe UI", 13),
                $labelBrush, $left + 20, $top + 30)
            return
        }

        $points = [System.Drawing.PointF[]]::new(128)
        for ($index = 0; $index -lt 128; $index++) {
            $x = $left + (($index / 127.0) * $plotWidth)
            $value = [Math]::Max(
                0, [Math]::Min(4095, $script:latestFrame.Pixels[$index]))
            $y = $top + $plotHeight -
                (($value / 4095.0) * $plotHeight)
            $points[$index] =
                [System.Drawing.PointF]::new([float]$x, [float]$y)
        }
        $graphics.DrawLines($tracePen, $points)

        foreach ($peakCenter in $script:latestFrame.PeakCenters) {
            $x = $left + (($peakCenter / 127.0) * $plotWidth)
            $value = $script:latestFrame.Pixels[$peakCenter]
            $y = $top + $plotHeight -
                (($value / 4095.0) * $plotHeight)
            $graphics.DrawEllipse(
                $peakPen, [float]($x - 5), [float]($y - 5), 10, 10)
            $graphics.DrawString(
                $peakCenter.ToString(), $font, $labelBrush,
                [float]($x + 7), [float]($y - 18))
        }

        $brightest = $script:latestFrame.BrightestPixel
        $brightestX = $left + (($brightest / 127.0) * $plotWidth)
        $graphics.DrawString(
            "brightest $brightest",
            $font, $labelBrush,
            [float][Math]::Min(
                $brightestX + 5, $left + $plotWidth - 110),
            [float]($top + $plotHeight - 24))
    }
    finally {
        $gridPen.Dispose()
        $axisPen.Dispose()
        $centerPen.Dispose()
        $tracePen.Dispose()
        $peakPen.Dispose()
        $labelBrush.Dispose()
        $mutedBrush.Dispose()
        $font.Dispose()
    }
})

$timer = [System.Windows.Forms.Timer]::new()
$timer.Interval = $IntervalMs
$timer.Add_Tick({
    $timer.Stop()
    try {
        $frame = Read-CcdFrame `
            ([int]$exposureControl.Value) `
            ([int]$thresholdControl.Value)
        $script:latestFrame = $frame
        $peakText = if ($frame.PeakCenters.Count -gt 0) {
            $frame.PeakCenters -join ","
        }
        else {
            "none"
        }
        $centerError = [Math]::Round(
            $frame.BrightestPixel - 63.5, 1)
        $statusLabel.ForeColor = [System.Drawing.SystemColors]::ControlText
        $statusLabel.Text = (
            ("frame {0}   min {1}   max {2}   span {3}   " +
             "peaks {4} [{5}]   brightest {6}:{7}   center error {8}px") -f
            $frame.Number, $frame.Minimum, $frame.Maximum, $frame.Span,
            $frame.PeakCount, $peakText, $frame.BrightestPixel,
            $frame.BrightestValue, $centerError)
        $plotPanel.Invalidate()
    }
    catch {
        $statusLabel.ForeColor = [System.Drawing.Color]::Firebrick
        $statusLabel.Text = "Capture error: $($_.Exception.Message)"
        $script:running = $false
        $toggleButton.Text = "Resume"
        Close-CcdSerial
    }
    finally {
        if ($script:running) {
            $timer.Start()
        }
    }
})

$toggleButton.Add_Click({
    $script:running = -not $script:running
    if ($script:running) {
        $toggleButton.Text = "Pause"
        $statusLabel.ForeColor = [System.Drawing.SystemColors]::ControlText
        $statusLabel.Text = "Resuming capture..."
        $timer.Start()
    }
    else {
        $toggleButton.Text = "Resume"
        $statusLabel.Text += "   PAUSED"
        $timer.Stop()
    }
})

$form.Add_KeyDown({
    param($sender, $eventArgs)
    if ($eventArgs.KeyCode -eq [System.Windows.Forms.Keys]::Space) {
        $toggleButton.PerformClick()
        $eventArgs.SuppressKeyPress = $true
    }
})

$form.Add_Shown({
    try {
        Open-CcdSerial
        $statusLabel.Text = "Connected to $Port; capturing..."
        $timer.Start()
    }
    catch {
        $statusLabel.ForeColor = [System.Drawing.Color]::Firebrick
        $statusLabel.Text = "Serial open error: $($_.Exception.Message)"
        $script:running = $false
        $toggleButton.Text = "Resume"
    }
})

$form.Add_FormClosed({
    $timer.Stop()
    $timer.Dispose()
    Close-CcdSerial
})

$form.Controls.Add($plotPanel)
$form.Controls.Add($hintLabel)
$form.Controls.Add($statusLabel)
$form.Controls.Add($toolbar)

[void][System.Windows.Forms.Application]::Run($form)
