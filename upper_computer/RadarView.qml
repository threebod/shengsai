import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

GridLayout {
    id: radar
    property var device
    property bool compact: width < 680
    property real viewportHeight: 680
    property var selectedPoint: null
    readonly property real rangeMm: Number(rangeChoice.currentText) * 1000
    readonly property int validPoints: {
        let n = 0
        for (const p of device.lidarPoints) if (p.distance > 0) ++n
        return n
    }
    columns: compact ? 1 : 3
    columnSpacing: 16; rowSpacing: 16

    component ScanButton: Button {
        id: button
        property bool primary: false
        implicitHeight: 44; implicitWidth: 108
        contentItem: Text {
            text: button.text; font: button.font
            color: !button.enabled ? "#A1A1A6" : button.primary ? "white" : "#1D1D1F"
            horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
        }
        background: Rectangle {
            radius: 10; color: !button.enabled ? "#EEEEF1" : button.down ? "#D8E8FF" : button.primary ? "#007AFF" : "white"
            border.color: button.primary ? "transparent" : "#D8D8DE"
        }
    }

    Pane {
        Layout.fillWidth: true; Layout.alignment: Qt.AlignTop; padding: 16
        Layout.preferredWidth: radar.compact ? -1 : 280
        background: Rectangle { color: "white"; radius: 18; border.color: "#E5E5EA" }
        contentItem: ColumnLayout {
            spacing: 12
            Label { text: "T-mini Plus 扫描"; font.pixelSize: 17; font.weight: Font.DemiBold }
            Flow {
                Layout.fillWidth: true; spacing: 8
                ScanButton {
                    objectName: "lidarStartButton"; text: "开始扫描"; primary: true
                    enabled: radar.device.connected && !radar.device.lidarRunning && !radar.device.lidarPending
                    onClicked: radar.device.setLidarScanning(true)
                }
                ScanButton {
                    objectName: "lidarStopButton"; text: "停止扫描"
                    enabled: radar.device.connected && radar.device.lidarRunning && !radar.device.lidarPending
                    onClicked: radar.device.setLidarScanning(false)
                }
            }
            Label {
                objectName: "lidarStatusLabel"; text: radar.device.lidarStatus
                Layout.fillWidth: true; wrapMode: Text.Wrap; color: "#656570"
            }
            Label {
                text: "有效点 " + radar.validPoints + " / " + radar.device.lidarPoints.length
                    + "    扫描 " + (radar.device.lidarPeriod > 0 ? (1000 / radar.device.lidarPeriod).toFixed(1) + " Hz" : "—")
                    + "    校验/接收异常 " + radar.device.lidarErrors
                    + (radar.device.lidarAge >= 0 ? "    扫描年龄 " + radar.device.lidarAge + " ms" : "")
                Layout.fillWidth: true; wrapMode: Text.Wrap; font.pixelSize: 12; color: "#656570"
            }
        }
    }

    Pane {
        Layout.fillWidth: true; Layout.alignment: Qt.AlignTop
        Layout.columnSpan: radar.compact ? 1 : 2
        Layout.preferredWidth: radar.compact ? -1 : 700
        padding: radar.compact ? 12 : 20
        background: Rectangle { color: "white"; radius: 18; border.color: "#E5E5EA" }
        contentItem: ColumnLayout {
            spacing: 12
            GridLayout {
                Layout.fillWidth: true; columns: radar.compact ? 2 : 4; columnSpacing: 12
                Label { text: "显示半径（m）"; Layout.fillWidth: true }
                ComboBox {
                    id: rangeChoice; objectName: "lidarRangeChoice"
                    model: ["1", "2", "4", "8", "12"]; currentIndex: 2
                    implicitHeight: 44; Layout.fillWidth: true; Layout.minimumWidth: 0
                    background: Rectangle { color: "#FAFAFC"; radius: 9; border.color: "#DDDEE5" }
                    onCurrentIndexChanged: plot.requestPaint()
                }
                CheckBox {
                    id: hideInterference; text: "隐藏干扰点"; implicitHeight: 44
                    Layout.fillWidth: true; font.pixelSize: 12
                    onCheckedChanged: plot.requestPaint()
                }
                CheckBox {
                    id: intensityColor; text: "按光强着色"; checked: true; implicitHeight: 44
                    Layout.fillWidth: true; font.pixelSize: 12
                    onCheckedChanged: plot.requestPaint()
                }
            }
            Rectangle {
                Layout.fillWidth: true; implicitHeight: Math.max(260, Math.min(width, radar.viewportHeight - 280, 540))
                radius: 12; color: "#101D2D"; clip: true
                Canvas {
                    id: plot; objectName: "lidarPlot"; anchors.fill: parent
                    readonly property real radius: Math.max(1, Math.min(width, height) / 2 - 30)
                    function pointVisible(p) {
                        return p.distance > 0 && p.distance <= radar.rangeMm
                            && !(hideInterference.checked && p.flag >= 2)
                    }
                    onWidthChanged: requestPaint()
                    onHeightChanged: requestPaint()
                    onPaint: {
                        const ctx = getContext("2d"), cx = width / 2, cy = height / 2
                        ctx.clearRect(0, 0, width, height)
                        ctx.font = "11px sans-serif"; ctx.lineWidth = 1
                        ctx.strokeStyle = "#2C4055"; ctx.fillStyle = "#92A5B8"
                        for (let i = 1; i <= 4; ++i) {
                            const r = radius * i / 4
                            ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.stroke()
                            ctx.fillText((radar.rangeMm * i / 4000).toFixed(1) + "m", cx + 4, cy - r + 13)
                        }
                        ctx.beginPath(); ctx.moveTo(cx - radius, cy); ctx.lineTo(cx + radius, cy)
                        ctx.moveTo(cx, cy - radius); ctx.lineTo(cx, cy + radius); ctx.stroke()
                        ctx.fillText("0°", cx - 8, cy - radius - 10)
                        ctx.fillText("90°", cx + radius + 3, cy + 4)
                        ctx.fillText("180°", cx - 15, cy + radius + 18)
                        ctx.fillText("270°", cx - radius - 29, cy + 4)
                        const scale = radius / radar.rangeMm
                        ctx.globalAlpha = radar.device.lidarFresh ? 1 : 0.3
                        for (const p of radar.device.lidarPoints) {
                            if (!pointVisible(p)) continue
                            ctx.fillStyle = p.flag >= 2 ? "#FF8B76" : intensityColor.checked
                                ? Qt.hsla(0.55 - p.intensity / 255 * 0.4, 0.85, 0.65, 1).toString() : "#57D7DD"
                            ctx.fillRect(cx + p.x * scale - 1.5, cy - p.y * scale - 1.5, 3, 3)
                        }
                        ctx.globalAlpha = 1
                        ctx.fillStyle = "#F0F5FA"; ctx.beginPath(); ctx.arc(cx, cy, 4, 0, Math.PI * 2); ctx.fill()
                        if (!radar.device.lidarFresh) {
                            ctx.fillStyle = "#C3CFDC"; ctx.textAlign = "center"
                            ctx.fillText(radar.device.lidarPoints.length ? "历史点云 · 当前无新鲜扫描" : "等待雷达点云", cx, height - 12)
                            ctx.textAlign = "left"
                        }
                    }
                    MouseArea {
                        id: pointMouse
                        anchors.fill: parent; hoverEnabled: true
                        function selectAt(mx, my) {
                            let nearest = null, best = 144
                            const scale = plot.radius / radar.rangeMm
                            for (const p of radar.device.lidarPoints) {
                                if (!plot.pointVisible(p)) continue
                                const dx = plot.width / 2 + p.x * scale - mx
                                const dy = plot.height / 2 - p.y * scale - my
                                const d = dx * dx + dy * dy
                                if (d < best) { best = d; nearest = p }
                            }
                            radar.selectedPoint = nearest
                        }
                        onPositionChanged: mouse => selectAt(mouse.x, mouse.y)
                        onClicked: mouse => selectAt(mouse.x, mouse.y)
                        onExited: radar.selectedPoint = null
                    }
                }
            }
            Label {
                objectName: "lidarPointDetails"; Layout.fillWidth: true; wrapMode: Text.Wrap
                text: radar.selectedPoint ? "角度 " + radar.selectedPoint.angle.toFixed(2) + "°  ·  距离 "
                    + radar.selectedPoint.distance + " mm  ·  光强 " + radar.selectedPoint.intensity
                    + "  ·  " + (radar.selectedPoint.flag === 2 ? "镜面反射干扰" : radar.selectedPoint.flag === 3 ? "环境光干扰" : "标记 " + radar.selectedPoint.flag)
                    : "鼠标移到点上或点击查看角度、距离、光强和干扰标记。"
                color: "#656570"; font.pixelSize: 12
            }
            Label {
                Layout.fillWidth: true; wrapMode: Text.Wrap; font.pixelSize: 12; color: "#86868B"
                text: "原点为雷达，0°朝上、角度顺时针；未做车头安装角标定。红色为干扰标记点，其他点按光强由蓝到黄显示；显示半径和隐藏选项仅影响本页。"
            }
        }
    }
    Connections {
        target: radar.device
        function onLidarChanged() { plot.requestPaint() }
        function onLidarPointsChanged() {
            if (pointMouse.containsMouse) pointMouse.selectAt(pointMouse.mouseX, pointMouse.mouseY)
            else radar.selectedPoint = null
            plot.requestPaint()
        }
    }
}
