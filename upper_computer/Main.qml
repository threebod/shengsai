import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ApplicationWindow {
    id: app
    width: 1280; height: 860; minimumWidth: 320; minimumHeight: 320
    visible: true; title: "省赛控制台"; color: "#F5F5F7"
    property int page: 0
    property string logText: ""
    property bool pickupDirty: false
    property bool ringsDirty: false
    property bool heightsDirty: false
    property bool scaleDirty: false
    property int obstacleMask: 0
    readonly property int obstacleCount: {
        let count = 0
        for (let bits = obstacleMask; bits; bits >>= 1) count += bits & 1
        return count
    }
    function toggleObstacle(region) {
        const bit = 1 << (region - 1)
        if (obstacleMask & bit) obstacleMask &= ~bit
        else if (obstacleCount < 2) obstacleMask |= bit
    }
    readonly property bool mobile: width - SafeArea.margins.left - SafeArea.margins.right < 760 || (Qt.platform.os === "android" && height < 500)
    readonly property int pageSpacing: mobile ? 14 : 20
    readonly property color blue: "#007AFF"
    readonly property bool motionReady: controller.connected && controller.confirmed && !controller.calibrating && !controller.axis6Testing && !controller.lateralTesting && !controller.straightTesting && !controller.turnTesting
    readonly property var pageNames: ["比赛", "圆环", "视觉", "手动", "雷达"]
    font.family: Qt.platform.os === "windows" ? "Microsoft YaHei UI" : "sans-serif"
    font.pixelSize: 14
    palette.window: "#F5F5F7"
    palette.base: "white"
    palette.button: "white"
    palette.buttonText: "#1D1D1F"
    palette.text: "#1D1D1F"
    palette.highlight: "#007AFF"
    onActiveChanged: if (!active) controller.pauseManual()
    onClosing: controller.pauseManual()
    Connections {
        target: controller
        function onLineReceived(line) { app.logText=(app.logText+line+"\n").slice(-40000) }
        function onTaskLogReceived(text) { app.logText=text }
    }

    component ActionButton: Button {
        id: ab
        property bool primary: false
        property bool danger: false
        implicitHeight: 44; implicitWidth: Math.max(96, contentItem.implicitWidth + 32)
        font.pixelSize: 14; font.weight: Font.Medium
        contentItem: Text { text: ab.text; font: ab.font; color: !ab.enabled ? "#A1A1A6" : ab.primary || ab.danger ? "white" : "#1D1D1F"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
        background: Rectangle { radius: 10; color: !ab.enabled ? "#EEEEF1" : ab.down ? "#D8E8FF" : ab.danger ? "#FF3B30" : ab.primary ? app.blue : "white"; border.color: ab.primary || ab.danger ? "transparent" : "#D8D8DE" }
    }
    component Card: Pane {
        id: card
        property string heading
        default property alias items: body.data
        padding: app.mobile ? 16 : 20; Layout.fillWidth: true
        background: Rectangle { color: "white"; radius: 18; border.color: "#E5E5EA" }
        contentItem: ColumnLayout {
            spacing: app.mobile ? 12 : 16
            Label { text: card.heading; font.pixelSize: 17; font.weight: Font.DemiBold; color: "#1D1D1F"; Layout.fillWidth: true }
            ColumnLayout { id: body; spacing: app.mobile ? 10 : 14; Layout.fillWidth: true }
        }
    }
    component Hint: Label { color: "#86868B"; wrapMode: Text.WordWrap; Layout.fillWidth: true; Layout.minimumWidth: 0; font.pixelSize: 12 }
    component NumberField: TextField {
        id: numberField
        property real minimum: -10000
        property real maximum: 10000
        property int decimals: 1
        readonly property real number: Number(text)
        property bool valid: acceptableInput && text.trim() !== "" && isFinite(number) && number >= minimum && number <= maximum
        Layout.fillWidth: true; implicitHeight: 44; selectByMouse: true
        inputMethodHints: Qt.ImhFormattedNumbersOnly
        validator: DoubleValidator { bottom: numberField.minimum; top: numberField.maximum; decimals: numberField.decimals; locale: "C"; notation: DoubleValidator.StandardNotation }
        color: valid ? "#1D1D1F" : "#D33B30"; font.pixelSize: 16
        background: Rectangle { color: "#FAFAFC"; radius: 9; border.color: parent.activeFocus ? app.blue : "#DDDEE5" }
    }
    component Choice: ComboBox {
        id: choice
        implicitHeight: 44; Layout.fillWidth: true; Layout.minimumWidth: 0
        background: Rectangle { color: "#FAFAFC"; radius: 9; border.color: choice.activeFocus ? app.blue : "#DDDEE5" }
        rightPadding: 36
        indicator: Canvas {
            implicitWidth: 12; implicitHeight: 8
            anchors.right: parent.right; anchors.rightMargin: 14; anchors.verticalCenter: parent.verticalCenter
            onPaint: {
                let ctx = getContext("2d")
                ctx.clearRect(0, 0, width, height)
                ctx.beginPath(); ctx.moveTo(1, 1); ctx.lineTo(width / 2, height - 1); ctx.lineTo(width - 1, 1)
                ctx.strokeStyle = "#86868B"; ctx.lineWidth = 2; ctx.stroke()
            }
        }
    }

    header: Pane {
        objectName: "topBar"
        padding: app.mobile ? 10 : 24
        topPadding: padding + app.SafeArea.margins.top
        leftPadding: (app.mobile ? 14 : 24) + app.SafeArea.margins.left
        rightPadding: (app.mobile ? 14 : 24) + app.SafeArea.margins.right
        background: Rectangle { color: "#FAFAFC"; border.color: "#E5E5EA" }
        contentItem: RowLayout {
            objectName: "topBarContent"; spacing: app.mobile ? 8 : 12
            ColumnLayout {
                Layout.fillWidth: true; spacing: 2
                Label { text: "省赛控制台"; font.pixelSize: 18; font.weight: Font.DemiBold; Layout.fillWidth: true; elide: Text.ElideRight }
                Label { visible: app.mobile; text: controller.connected ? "已连接" : "未连接"; color: controller.connected ? "#30B451" : "#86868B"; font.pixelSize: 11 }
            }
            Rectangle { visible: !app.mobile; width: 8; height: 8; radius: 4; color: controller.connected ? "#30B451" : "#A1A1A6" }
            Label { visible: !app.mobile; text: controller.connected ? controller.connectionName + " 已连接" : "未连接"; color: "#656570"; font.pixelSize: 12 }
            ActionButton { text: "日志"; implicitWidth: app.mobile ? 56 : 64; onClicked: logDialog.open() }
            ActionButton { text: "连接"; implicitWidth: app.mobile ? 56 : 64; onClicked: connectionDialog.open() }
        }
    }
    footer: Pane {
        objectName: "bottomBar"
        padding: 0
        bottomPadding: app.SafeArea.margins.bottom
        leftPadding: app.SafeArea.margins.left
        rightPadding: app.SafeArea.margins.right
        background: Rectangle { color: "white" }
        contentItem: ColumnLayout {
            objectName: "bottomBarContent"
            spacing: 0
            Rectangle { Layout.fillWidth: true; height: 1; color: "#E5E5EA" }
            Pane {
                Layout.fillWidth: true; padding: app.mobile ? 8 : 20
                background: Rectangle { color: "#FAFAFC" }
                contentItem: RowLayout {
                    Label { visible: !app.mobile; text: controller.status; color: "#686875"; elide: Text.ElideRight; Layout.fillWidth: true }
                    ActionButton { text: "停止"; Layout.fillWidth: app.mobile; enabled: controller.connected; onClicked: controller.stop() }
                    ActionButton { text: "急停"; danger: true; Layout.fillWidth: app.mobile; enabled: controller.connected; onClicked: controller.stop(true) }
                }
            }
            TabBar {
                objectName: "navigationBar"
                visible: app.mobile; Layout.fillWidth: true
                currentIndex: app.page
                onCurrentIndexChanged: app.page = currentIndex
                background: Rectangle { color: "white" }
                Repeater {
                    model: app.pageNames
                    TabButton {
                        id: navTab
                        required property int index; required property string modelData
                        text: modelData; implicitHeight: 48
                        background: Rectangle { color: "white"; Rectangle { anchors.top: parent.top; width: parent.width; height: 2; color: app.page === navTab.index ? app.blue : "transparent" } }
                        contentItem: Text { text: navTab.text; font: navTab.font; color: app.page === navTab.index ? app.blue : "#86868B"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                    }
                }
            }
        }
    }
    RowLayout {
        anchors.fill: parent; spacing: 0
        Pane {
            visible: !app.mobile; Layout.fillHeight: true; Layout.preferredWidth: 152; padding: 12
            background: Rectangle { color: "#FAFAFC"; border.color: "#E5E5EA" }
            ColumnLayout {
                anchors.fill: parent; spacing: 8
                Repeater {
                    model: app.pageNames
                    delegate: ActionButton {
                        required property int index; required property string modelData
                        text: modelData; primary: app.page === index; Layout.fillWidth: true
                        onClicked: app.page = index
                    }
                }
                Item { Layout.fillHeight: true }
                Label { text: "预发布 0.1.0"; color: "#86868B"; font.pixelSize: 11 }
            }
        }
        ScrollView {
            id: scroll; objectName: "pageScroll"
            Layout.fillWidth: true; Layout.fillHeight: true; clip: true
            contentWidth: availableWidth
            ColumnLayout {
                width: scroll.availableWidth; spacing: app.pageSpacing
                Item { height: app.mobile ? 0 : 4 }
                ColumnLayout {
                    Layout.fillWidth: true; Layout.leftMargin: app.mobile ? 14 : 28; Layout.rightMargin: app.mobile ? 14 : 28; spacing: app.pageSpacing
                    Label { text: app.pageNames[app.page]; font.pixelSize: app.mobile ? 24 : 28; font.weight: Font.DemiBold; color: "#1D1D1F" }
                    Hint { visible: app.mobile; text: controller.status }
                    RowLayout {
                        Layout.fillWidth: true; visible: controller.connected && !controller.confirmed
                        Hint { text: "运动前请确认电机映射、方向、限位、急停及机构零点。" }
                        ActionButton { text: "确认安全与零点"; onClicked: safetyDialog.open() }
                    }
                    GridLayout {
                        uniformCellWidths: true
                        visible: app.page === 0; columns: app.mobile ? 1 : 2; columnSpacing: app.pageSpacing; rowSpacing: app.pageSpacing; Layout.fillWidth: true
                        Card {
                            heading: "比赛设置"; Layout.alignment: Qt.AlignTop
                            Label { text: "启停区"; color: "#656570" }
                            Choice { id: zone; model: ["1号启停区", "2号启停区"] }
                            Label { text: "障碍位置"; color: "#656570" }
                            Hint { objectName: "obstacleSelectionHint"; text: "可选0～2个区域 · 已选 " + app.obstacleCount + "/2；再次点击可取消" }
                            Item {
                                id: obstacleMap; objectName: "obstacleMap"
                                Layout.fillWidth: true; implicitHeight: Math.max(width, 320)
                                readonly property real sx: width / 2400
                                readonly property real sy: height / 2400
                                Rectangle { anchors.fill: parent; color: "#E5E5EA"; radius: 4 }
                                Repeater {
                                    model: [[550, 550], [1400, 550], [550, 1400], [1400, 1400]]
                                    delegate: Rectangle {
                                        required property var modelData
                                        x: modelData[0] * obstacleMap.sx; y: (2400-modelData[1]-450) * obstacleMap.sy
                                        width: 450 * obstacleMap.sx; height: 450 * obstacleMap.sy; color: "#FAFAE9"
                                    }
                                }
                                Repeater {
                                    model: controller.obstacleRegions
                                    delegate: Button {
                                        id: regionButton
                                        required property int index; required property var modelData
                                        objectName: "obstacleRegion" + (index + 1)
                                        readonly property bool selected: (app.obstacleMask & (1 << index)) !== 0
                                        x: modelData.x0 * obstacleMap.sx + 1; y: (2400-modelData.y1) * obstacleMap.sy + 1
                                        width: (modelData.x1-modelData.x0) * obstacleMap.sx - 2
                                        height: (modelData.y1-modelData.y0) * obstacleMap.sy - 2
                                        enabled: !controller.busy
                                        text: String(index + 1)
                                        Accessible.name: "障碍区域" + text + (selected ? "，已选" : "，未选")
                                        onClicked: app.toggleObstacle(index + 1)
                                        contentItem: Text { text: regionButton.text; color: regionButton.selected ? "white" : "#1D1D1F"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; font.pixelSize: 16 }
                                        background: Rectangle { color: regionButton.selected ? "#009B83" : "white"; border.color: "#EF6060"; radius: 3 }
                                    }
                                }
                                Repeater {
                                    model: [
                                        {x: 1000, y: 2080, w: 400, h: 320, label: "原料"},
                                        {x: 1000, y: 0, w: 400, h: 300, label: "粗加工"},
                                        {x: 0, y: 1000, w: 300, h: 400, label: "暂\n存"},
                                        {x: 1950, y: 2100, w: 450, h: 300, label: "启停1"},
                                        {x: 1950, y: 0, w: 450, h: 300, label: "启停2"}
                                    ]
                                    delegate: Label {
                                        required property var modelData
                                        x: modelData.x * obstacleMap.sx; y: (2400-modelData.y-modelData.h) * obstacleMap.sy
                                        width: modelData.w * obstacleMap.sx; height: modelData.h * obstacleMap.sy
                                        text: modelData.label; font.pixelSize: 10
                                        horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                                        color: modelData.label.indexOf("启停") === 0 ? "white" : "#656570"
                                        background: Rectangle { color: modelData.label.indexOf("启停") === 0 ? "#29A7D3" : "transparent" }
                                    }
                                }
                            }
                            Hint { text: "选中的红框整体禁行。只修改选区不会驱动机构，启动时下发主控。" }
                            Hint { text: "比赛沿用主控保存的圆环落点、视觉比例和既有动作组。" }
                        }
                        Card {
                            heading: "当前任务"; Layout.alignment: Qt.AlignTop
                            Label { text: controller.task; color: app.blue; font.pixelSize: 20; wrapMode: Text.Wrap; Layout.fillWidth: true }
                            Hint { text: "扫码 → 取料 → 粗加工 → 暂存 · 共两轮" }
                            ActionButton { objectName: "missionStartButton"; text: "开始比赛"; primary: true; Layout.fillWidth: true; enabled: app.motionReady && !controller.busy && app.obstacleCount <= 2; onClicked: controller.run(1, zone.currentIndex + 1, app.obstacleMask) }
                            ActionButton { objectName: "missionPrepareButton"; text: "离线按钮启动准备"; Layout.fillWidth: true; enabled: app.motionReady && !controller.busy && app.obstacleCount <= 2; onClicked: controller.run(0, zone.currentIndex + 1, app.obstacleMask) }
                            Hint { text: "离线准备后，PC9 短按选择启停区1，长按选择启停区2。" }
                        }
                        Card {
                            heading: "单项执行"; Layout.columnSpan: app.mobile ? 1 : 2
                            Flow {
                                Layout.fillWidth: true; spacing: 10
                                ActionButton { objectName: "routeStartButton"; text: "仅跑图"; enabled: app.motionReady && !controller.busy && app.obstacleCount <= 2; onClicked: controller.run(2, zone.currentIndex+1, app.obstacleMask) }
                                ActionButton { text: "取料动作组"; enabled: app.motionReady && !controller.busy; onClicked: controller.runAction(0) }
                                ActionButton { text: "粗加工动作组"; enabled: app.motionReady && !controller.busy; onClicked: controller.runAction(1) }
                                ActionButton { text: "暂存动作组"; enabled: app.motionReady && !controller.busy; onClicked: controller.runAction(2) }
                            }
                        }
                    }
                    ColumnLayout {
                        visible: app.page === 1; Layout.fillWidth: true; spacing: app.pageSpacing
                        Hint { text: "先读取，再修改并固化。保存参数不会驱动机构。" }
                        GridLayout {
                        uniformCellWidths: true
                            columns: app.mobile ? 1 : 3; Layout.fillWidth: true; columnSpacing: 16; rowSpacing: 16
                            Repeater {
                                id: ringRepeater; model: 3
                                delegate: Card {
                                    required property int index
                                    property alias horizontal: ringH.number
                                    property alias turret: ringT.number
                                    readonly property bool valid: ringH.valid && ringT.valid
                                    heading: (index+1)+"号圆环"
                                    Label { text: "水平位置 mm"; color: "#656570" }
                                    NumberField { id: ringH; minimum: -122; maximum: 85; enabled: controller.ringsRead && !controller.busy && !controller.parametersPending; placeholderText: "待读取"; onTextEdited: app.ringsDirty=true }
                                    Label { text: "云台角度 °"; color: "#656570" }
                                    NumberField { id: ringT; minimum: 0; maximum: 360; enabled: controller.ringsRead && !controller.busy && !controller.parametersPending; placeholderText: "待读取"; onTextEdited: app.ringsDirty=true }
                                    Connections {
                                        target: controller
                                        function onRequestAccepted(command) {
                                            if(command === 16 && controller.rings.length === 3) {
                                                ringH.text = controller.rings[index].horizontal.toFixed(1)
                                                ringT.text = controller.rings[index].turret.toFixed(1)
                                                app.ringsDirty=false
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        Card {
                            heading: "参数状态"
                            Hint { text: app.ringsDirty ? "参数已修改，待固化" : controller.ringState }
                            RowLayout {
                                Layout.fillWidth: true
                                ActionButton { text: "读取主控"; Layout.fillWidth: true; enabled: controller.connected && !controller.busy && !controller.parametersPending; onClicked: {app.ringsDirty=false;controller.readRings()} }
                                ActionButton {
                                    text: "固化三组参数"; primary: true; Layout.fillWidth: true; enabled: controller.connected && controller.ringsRead && !controller.busy && !controller.parametersPending
                                    onClicked: {
                                        let values=[]
                                        for(let i=0;i<3;i++) {let item=ringRepeater.itemAt(i); if(!item.valid) return;values.push({horizontal:item.horizontal,turret:item.turret})}
                                        app.ringsDirty=false;controller.saveRings(values)
                                    }
                                }
                            }
                        }
                        GridLayout {
                        uniformCellWidths: true
                            columns: app.mobile ? 1 : 3; Layout.fillWidth: true; columnSpacing: 16; rowSpacing: 16
                            Repeater {
                                id: pickupRepeater; model: 3
                                delegate: Card {
                                    required property int index
                                    property alias horizontal: pickupH.number
                                    property alias turret: pickupT.number
                                    readonly property bool valid: pickupH.valid && pickupT.valid
                                    heading: ["取料区 · 下方", "取料区 · 左上", "取料区 · 右上"][index]
                                    Label { text: "水平位置 mm"; color: "#656570" }
                                    NumberField { id: pickupH; minimum: -122; maximum: 85; enabled: controller.pickupRead && !controller.busy && !controller.parametersPending; placeholderText: "待读取"; onTextEdited: app.pickupDirty=true }
                                    Label { text: "云台角度 °"; color: "#656570" }
                                    NumberField { id: pickupT; minimum: 0; maximum: 360; enabled: controller.pickupRead && !controller.busy && !controller.parametersPending; placeholderText: "待读取"; onTextEdited: app.pickupDirty=true }
                                    Connections {
                                        target: controller
                                        function onRequestAccepted(command) {
                                            if(command === 24 && controller.pickup.length === 3) {
                                                pickupH.text = controller.pickup[index].horizontal.toFixed(1)
                                                pickupT.text = controller.pickup[index].turret.toFixed(1)
                                                app.pickupDirty=false
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        Card {
                            heading: "取料区参数状态"
                            Hint { text: app.pickupDirty ? "参数已修改，待固化" : controller.pickupState }
                            RowLayout {
                                Layout.fillWidth: true
                                ActionButton { objectName: "pickupReadButton"; text: "读取取料区"; Layout.fillWidth: true; enabled: controller.connected && !controller.busy && !controller.parametersPending; onClicked: {app.pickupDirty=false;controller.readPickup()} }
                                ActionButton {
                                    objectName: "pickupSaveButton"; text: "固化取料区参数"; primary: true; Layout.fillWidth: true; enabled: controller.connected && controller.pickupRead && !controller.busy && !controller.parametersPending
                                    onClicked: {
                                        let values=[]
                                        for(let i=0;i<3;i++) {let item=pickupRepeater.itemAt(i); if(!item.valid) return;values.push({horizontal:item.horizontal,turret:item.turret})}
                                        app.pickupDirty=false;controller.savePickup(values)
                                    }
                                }
                            }
                        }
                        Card {
                            objectName: "placementHeightsCard"
                            heading: "放置深度"
                            Hint { text: "从升降零位向下的深度，数值越大越往下。先读取，再修改并固化；保存不会驱动机构。" }
                            GridLayout {
                                columns: app.mobile ? 1 : 2; Layout.fillWidth: true; columnSpacing: 16; rowSpacing: 12
                                Repeater {
                                    id: heightsRepeater
                                    model: ["一层放置 mm", "二层放置 mm", "原料落盘 mm", "回收落盘 mm"]
                                    delegate: ColumnLayout {
                                        required property int index
                                        required property string modelData
                                        property alias depth: heightField.number
                                        readonly property bool valid: heightField.valid
                                        Layout.fillWidth: true
                                        Label { text: modelData; color: "#656570" }
                                        NumberField {
                                            id: heightField; minimum: 0; maximum: 150; placeholderText: "待读取"
                                            enabled: controller.heightsRead && !controller.busy && !controller.parametersPending
                                            onTextEdited: app.heightsDirty=true
                                        }
                                        Connections {
                                            target: controller
                                            function onRequestAccepted(command) {
                                                if(command === 21 && controller.placementHeights.length === 4) {
                                                    heightField.text=controller.placementHeights[index].toFixed(1)
                                                    app.heightsDirty=false
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            Hint { text: app.heightsDirty ? "深度已修改，待固化" : controller.heightsState }
                            RowLayout {
                                Layout.fillWidth: true
                                ActionButton {
                                    objectName: "heightsReadButton"; text: "读取放置深度"; Layout.fillWidth: true
                                    enabled: controller.connected && !controller.busy && !controller.parametersPending
                                    onClicked: { app.heightsDirty=false; controller.readHeights() }
                                }
                                ActionButton {
                                    objectName: "heightsSaveButton"; text: "固化放置深度"; primary: true; Layout.fillWidth: true
                                    enabled: controller.connected && controller.heightsRead && !controller.busy && !controller.parametersPending
                                    onClicked: {
                                        let values=[]
                                        for(let i=0;i<4;i++) { let item=heightsRepeater.itemAt(i); if(!item.valid) return; values.push(item.depth) }
                                        app.heightsDirty=false; controller.saveHeights(values)
                                    }
                                }
                            }
                        }
                    }
                    GridLayout {
                        uniformCellWidths: true
                        visible: app.page === 2; columns: app.mobile ? 1 : 2; Layout.fillWidth: true; columnSpacing: app.pageSpacing; rowSpacing: app.pageSpacing
                        Card {
                            heading: "2号圆环 · 自动标定"; Layout.alignment: Qt.AlignTop
                            Hint { text: "自动采样五个稳定中心，小车以低速分别前后、左右往返移动 20 mm。" }
                            BusyIndicator { running: controller.calibrating; visible: running; Layout.alignment: Qt.AlignHCenter }
                            ActionButton { text: controller.calibrating ? "取消标定" : "开始自动标定"; primary: true; Layout.fillWidth: true; enabled: controller.connected && (controller.calibrating || (app.motionReady && !controller.busy)); onClicked: controller.calibrating ? controller.stop() : controller.calibrate() }
                            ActionButton { text: "对准2号圆环"; Layout.fillWidth: true; enabled: app.motionReady && !controller.busy; onClicked: controller.alignRing() }
                            Hint { text: "标定前将机构放在圆环观察姿态，并确认往返空间。失败不会覆盖原比例。" }
                        }
                        Card {
                            heading: "视觉比例"; Layout.alignment: Qt.AlignTop
                            Label { text: "前后 mm/像素"; color: "#656570" }
                            NumberField { id: scaleF; minimum: 0.05; maximum: 2; decimals: 3; text: "1.111"; enabled: !controller.busy && !controller.calibrating && !controller.parametersPending; onTextEdited: app.scaleDirty=true }
                            Label { text: "左右 mm/像素"; color: "#656570" }
                            NumberField { id: scaleR; minimum: 0.05; maximum: 2; decimals: 3; text: "1.143"; enabled: !controller.busy && !controller.calibrating && !controller.parametersPending; onTextEdited: app.scaleDirty=true }
                            Hint { text: app.scaleDirty ? "比例已修改，待应用或保存" : controller.scaleState }
                            RowLayout {
                                Layout.fillWidth: true
                                ActionButton { text: "读取"; Layout.fillWidth: true; enabled: controller.connected && !controller.busy && !controller.calibrating && !controller.parametersPending; onClicked: {app.scaleDirty=false;controller.readScale()} }
                                ActionButton { text: "应用"; Layout.fillWidth: true; enabled: controller.connected && !controller.busy && !controller.calibrating && !controller.parametersPending && scaleF.valid && scaleR.valid; onClicked: {app.scaleDirty=false;controller.applyScale(scaleF.number,scaleR.number,false)} }
                            }
                            ActionButton { text: "保存到主控"; primary: true; Layout.fillWidth: true; enabled: controller.connected && !controller.busy && !controller.calibrating && !controller.parametersPending && scaleF.valid && scaleR.valid; onClicked: {app.scaleDirty=false;controller.applyScale(scaleF.number,scaleR.number,true)} }
                            Hint { text: "显式保存后断电保留，离线比赛继续使用。" }
                            Connections { target: controller; function onRequestAccepted(command) { if(command>=18 && command<=20) {scaleF.text=controller.forwardScale.toFixed(3);scaleR.text=controller.rightScale.toFixed(3);app.scaleDirty=false} } }
                        }
                    }
                    ColumnLayout {
                        visible: app.page === 3; Layout.fillWidth: true; spacing: app.pageSpacing
                        Card {
                            heading: "舵机指令角度"
                            Repeater {
                                model: ["夹爪", "转盘", "云台"]
                                delegate: RowLayout {
                                    required property int index; required property string modelData
                                    Layout.fillWidth: true; spacing: 12
                                    Label { text: modelData; Layout.preferredWidth: 40; color: "#656570" }
                                    Slider {
                                        id: servoSlider; Layout.fillWidth: true; from: 0; to: index===2 ? 360 : 270; stepSize: 0.1; enabled: app.motionReady
                                        onMoved: {servoNumber.text=value.toFixed(1);controller.setServo(index+2,value)}
                                        onPressedChanged: if(!pressed && enabled) controller.setServo(index+2,value,true)
                                    }
                                    NumberField {
                                        id: servoNumber; Layout.fillWidth: false; Layout.preferredWidth: app.mobile ? 76 : 110; minimum: 0; maximum: index===2 ? 360 : 270; text: "0.0"; enabled: app.motionReady
                                        onEditingFinished: if(valid){servoSlider.value=number;controller.setServo(index+2,number,true)}
                                    }
                                    Connections { target: controller; function onChanged() {if(!servoSlider.pressed && !servoNumber.activeFocus){servoSlider.value=controller.servoAngles[index];servoNumber.text=controller.servoAngles[index].toFixed(1)}} }
                                }
                            }
                            Hint { text: "显示主控指令角度，无舵机实际角度反馈；云台手动调整后需重新确认机构坐标。" }
                        }
                        GridLayout {
                        uniformCellWidths: true
                            Layout.fillWidth: true; columns: app.mobile ? 1 : 2; columnSpacing: app.pageSpacing; rowSpacing: app.pageSpacing
                            Card {
                                heading: "机构微调"
                                Choice { id: jogStep; model: ["0.1 mm", "1 mm", "5 mm"]; currentIndex: 1 }
                                Repeater {
                                    model: ["5号 · 升降", "6号 · 水平"]
                                    delegate: RowLayout {
                                        required property int index; required property string modelData
                                        Layout.fillWidth: true
                                        Label { text: modelData; Layout.fillWidth: true; color: "#656570" }
                                        ActionButton { text: "−"; implicitWidth: 52; enabled: app.motionReady && controller.mechanismValid && !controller.busy; onClicked: controller.jogAxis(index+5,-[0.1,1,5][jogStep.currentIndex]) }
                                        ActionButton { text: "+"; implicitWidth: 52; enabled: app.motionReady && controller.mechanismValid && !controller.busy; onClicked: controller.jogAxis(index+5,[0.1,1,5][jogStep.currentIndex]) }
                                    }
                                }
                                ActionButton { text: "重新确认物理零点"; enabled: controller.connected && !controller.busy; onClicked: safetyDialog.open() }
                                ActionButton {
                                    objectName: "axis6TestButton"; text: controller.axis6Testing ? "6号轴测试中…" : "6号轴往返测试"
                                    Layout.fillWidth: true
                                    enabled: app.motionReady && controller.mechanismValid && !controller.busy
                                    onClicked: controller.testAxis6()
                                }
                                Hint { text: "正向5mm → 返回起点，30RPM；请留足行程。切页或失焦停止，目标、实际位置和耗时见日志。" }
                                Hint { text: controller.axis6TestState }
                                Hint { visible: controller.connected && !controller.mechanismValid; text: "机构坐标已失效，请先将机构实际回到零点，再重新确认。" }
                            }
                            Card {
                                heading: "底盘点动"; Layout.alignment: Qt.AlignTop
                                ActionButton { text: "按住前进"; primary: true; Layout.fillWidth: true; enabled: app.motionReady; onPressed: controller.drive(1); onReleased: controller.stop(); onCanceled: controller.stop() }
                                ActionButton { text: "按住后退"; primary: true; Layout.fillWidth: true; enabled: app.motionReady; onPressed: controller.drive(-1); onReleased: controller.stop(); onCanceled: controller.stop() }
                                Hint { text: "松开、切换页面或窗口失焦即停；主控另有点动指令超时保护。" }
                            }
                            Card {
                                objectName: "turnTestCard"; heading: "90°原地转向测试"
                                RowLayout {
                                    Layout.fillWidth: true
                                    ActionButton { objectName: "turnLeftButton"; text: "左转90°"; primary: true; Layout.fillWidth: true; enabled: app.motionReady && !controller.busy; onClicked: controller.testTurn(1) }
                                    ActionButton { objectName: "turnRightButton"; text: "右转90°"; primary: true; Layout.fillWidth: true; enabled: app.motionReady && !controller.busy; onClicked: controller.testTurn(-1) }
                                }
                                ActionButton { objectName: "turnStopButton"; text: "停止转向测试"; Layout.fillWidth: true; enabled: controller.connected && controller.turnTesting; onClicked: controller.stop() }
                                Hint { text: "从当前朝向原地转90°，复用正式转向PID和加减速；±0.5°内停车，稳定200ms并确认新IMU数据后完成。" }
                                Hint { text: "记录每个新IMU角度及轮速指令变化。请左右各测3次，每次立即读取日志并注明载荷；下一次测试会清空上一趟记录。" }
                                Hint { text: "不自动转回、不执行取放动作；切页、失焦或断连停止。" }
                                Hint { text: controller.turnTestState }
                            }
                            Card {
                                objectName: "straightTestCard"; heading: "1680mm直行测试"
                                ActionButton {
                                    objectName: "straightTestButton"; text: controller.straightTesting ? "直行测试中…" : "直行1680mm并记录角度"
                                    primary: true; Layout.fillWidth: true
                                    enabled: app.motionReady && !controller.busy
                                    onClicked: controller.testStraight()
                                }
                                ActionButton { objectName: "straightStopButton"; text: "停止直行测试"; Layout.fillWidth: true; enabled: controller.connected && controller.straightTesting; onClicked: controller.stop() }
                                Hint { text: "沿当前车头方向前进1680mm，240RPM，沿用正式直线加减速和航向PID；不转向、不执行取放动作。" }
                                Hint { text: "每100ms记录角度和纠偏指令。结束后请测量实际侧偏，并在“日志”中读取上次任务日志；开始下次测试会清空记录。" }
                                Hint { text: "切页、失焦或断连停止；完成表示指令距离估算结束，请留足直线距离。" }
                                Hint { text: controller.straightTestState }
                            }
                            Card {
                                objectName: "lateralTestCard"; heading: "左右横移测距"
                                Hint { text: "测试距离（mm）" }
                                NumberField { id: lateralDistance; objectName: "lateralTestDistance"; minimum: 100; maximum: 2000; decimals: 0; text: "1000"; enabled: !controller.lateralTesting }
                                RowLayout {
                                    Layout.fillWidth: true
                                    ActionButton { objectName: "lateralLeftButton"; text: "向左测试"; Layout.fillWidth: true; enabled: app.motionReady && !controller.busy && lateralDistance.valid; onClicked: controller.testLateral(-1,lateralDistance.number) }
                                    ActionButton { objectName: "lateralRightButton"; text: "向右测试"; Layout.fillWidth: true; enabled: app.motionReady && !controller.busy && lateralDistance.valid; onClicked: controller.testLateral(1,lateralDistance.number) }
                                }
                                ActionButton { text: "停止横移"; Layout.fillWidth: true; enabled: controller.connected && controller.lateralTesting; onClicked: controller.stop() }
                                Hint { text: "按车体左右单次移动100～2000mm。沿用正式横移速度（最高150RPM）、加减速、航向保持与横移比例；请用尺测实际距离。" }
                                Hint { text: "切页、失焦或断连停止；完成表示指令距离估算结束，不是实测到位。" }
                                Hint { text: controller.lateralTestState }
                            }
                        }
                    }
                }
                RadarView {
                    objectName: "radarPage"; device: controller; visible: app.page === 4
                    viewportHeight: scroll.availableHeight
                    Layout.fillWidth: true
                    Layout.leftMargin: app.mobile ? 14 : 28; Layout.rightMargin: app.mobile ? 14 : 28
                }
                Item { height: 12 }
            }
        }
    }
    onPageChanged: {
        controller.pauseManual()
        if (page === 4) Qt.callLater(() => { scroll.contentItem.contentY = 0 })
    }
    component SafeDialog: Dialog {
        parent: Overlay.overlay
        readonly property real safeWidth: parent.width - parent.SafeArea.margins.left - parent.SafeArea.margins.right
        readonly property real safeHeight: parent.height - parent.SafeArea.margins.top - parent.SafeArea.margins.bottom
        width: Math.min(safeWidth - 32, 480)
        height: Math.min(implicitHeight, safeHeight - 32)
        x: parent.SafeArea.margins.left + (safeWidth - width) / 2
        y: parent.SafeArea.margins.top + (safeHeight - height) / 2
        modal: true
    }
    SafeDialog {
        id: connectionDialog; objectName: "connectionDialog"; title: "连接主控"
        standardButtons: Dialog.Close
        contentItem: ScrollView {
            id: connectionScroll
            clip: true; contentWidth: availableWidth
            ColumnLayout {
                width: connectionScroll.availableWidth; spacing: 16
                Hint { text: Qt.platform.os === "android" ? "选择 HC-05 经典蓝牙设备。" : "USB 串口可直接连接；HC-05 请先在 Windows 配对，再选择对应的出站 COM 串口。两种方式互斥接入 UART5。" }
                Choice { id: deviceChoice; model: controller.devices; textRole: "name"; valueRole: "address" }
                Flow {
                    Layout.fillWidth: true; Layout.minimumWidth: 0; spacing: 8
                    ActionButton { text: "扫描设备"; onClicked: controller.scan() }
                    ActionButton { text: "连接"; primary: true; enabled: deviceChoice.currentIndex>=0; onClicked: {controller.connectDevice(deviceChoice.currentValue);connectionDialog.close()} }
                    ActionButton { text: "断开"; enabled: controller.connected; onClicked: {controller.stop();controller.disconnectDevice()} }
                }
                Hint { text: controller.status }
            }
        }
    }
    SafeDialog {
        id: logDialog; objectName: "logDialog"; title: "运行日志"
        width: Math.min(safeWidth-32,800); height: Math.min(safeHeight-32,600)
        standardButtons: Dialog.Close
        contentItem: ColumnLayout {
            ActionButton { objectName: "taskLogReadButton"; text: controller.logsLoading ? "正在读取…" : "读取上次任务日志"; enabled: controller.connected && !controller.logsLoading; onClicked: controller.readTaskLog() }
            Hint { text: controller.logsState }
            ScrollView { Layout.fillWidth: true; Layout.fillHeight: true; TextArea { text: app.logText || "尚无主控日志"; readOnly: true; selectByMouse: true; wrapMode: TextEdit.WrapAnywhere; font.family: "monospace"; font.pixelSize: 12 } }
        }
    }
    SafeDialog {
        id: safetyDialog; objectName: "safetyDialog"; title: "确认安全条件与物理零点"
        standardButtons: Dialog.Cancel | Dialog.Ok
        onAccepted: if(safetyCheck.checked)controller.confirmZero()
        onOpened: safetyCheck.checked=false
        contentItem: ScrollView {
            id: safetyScroll
            clip: true; contentWidth: availableWidth
            ColumnLayout {
                width: safetyScroll.availableWidth; spacing: 16
                Hint { text: "仅当升降轴和水平轴实际位于零点时确认。本操作将两轴软件坐标设为零，不会自动回零。" }
                CheckBox { id: safetyCheck; Layout.fillWidth: true; Layout.minimumWidth: 0; text: "已确认映射、方向、机械限位、急停，以及两轴物理零点"; contentItem: Text { text: safetyCheck.text; font: safetyCheck.font; wrapMode: Text.Wrap; leftPadding: safetyCheck.indicator.width + 8; verticalAlignment: Text.AlignVCenter } }
                Hint { text: "确认仅对当前连接有效；急停或坐标失效后需重新检查。" }
            }
        }
        Component.onCompleted: standardButton(Dialog.Ok).enabled=Qt.binding(function(){return safetyCheck.checked})
    }
}
