import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

// Holding a button (or a key while the panel has focus) moves the robot;
// releasing it stops. Commands are published at 10 Hz while active.
ColumnLayout {
  id: root
  Layout.minimumWidth: 300
  Layout.minimumHeight: 480
  anchors.fill: parent
  anchors.margins: 8
  spacing: 8
  focus: true

  // Scales -1, 0, 1 on each axis from buttons and keys
  property int sx: 0
  property int sy: 0
  property int sw: 0
  property var keys: ({})
  property bool moving: sx !== 0 || sy !== 0 || sw !== 0

  function updateFromKeys() {
    var k = keys
    sx = (k[Qt.Key_W] ? 1 : 0) - (k[Qt.Key_S] ? 1 : 0)
    sy = (k[Qt.Key_A] ? 1 : 0) - (k[Qt.Key_D] ? 1 : 0)
    sw = (k[Qt.Key_Q] ? 1 : 0) - (k[Qt.Key_E] ? 1 : 0)
  }

  function send() {
    OmniTeleop.Publish(sx * linSlider.value, sy * linSlider.value, sw * angSlider.value)
  }

  onMovingChanged: send()   // sends the first command, or the zero on release

  Timer {
    interval: 100
    repeat: true
    running: root.moving
    onTriggered: root.send()
  }

  Keys.onPressed: function(event) {
    if (event.isAutoRepeat) return
    var k = keys; k[event.key] = true; keys = k
    updateFromKeys(); event.accepted = true
  }
  Keys.onReleased: function(event) {
    if (event.isAutoRepeat) return
    var k = keys; delete k[event.key]; keys = k
    updateFromKeys(); event.accepted = true
  }

  // --- Topic -------------------------------------------------------------
  RowLayout {
    Layout.fillWidth: true
    Label { text: "Topic" }
    TextField {
      Layout.fillWidth: true
      text: OmniTeleop.topic
      selectByMouse: true
      onEditingFinished: { OmniTeleop.SetTopic(text); root.forceActiveFocus() }
    }
  }

  // --- 8-direction pad ---------------------------------------------------------
  // [vx, vy] for each button; positive vy = left (ROS convention)
  GridLayout {
    Layout.alignment: Qt.AlignHCenter
    columns: 3
    rowSpacing: 4
    columnSpacing: 4

    Repeater {
      model: [
        { t: "↖", x: 1, y: 1 },  { t: "↑", x: 1, y: 0 },  { t: "↗", x: 1, y: -1 },
        { t: "←", x: 0, y: 1 },  { t: "■", x: 0, y: 0 },  { t: "→", x: 0, y: -1 },
        { t: "↙", x: -1, y: 1 }, { t: "↓", x: -1, y: 0 }, { t: "↘", x: -1, y: -1 }
      ]
      delegate: Button {
        text: modelData.t
        font.pixelSize: 26
        Layout.preferredWidth: 70
        Layout.preferredHeight: 60
        onPressedChanged: {
          root.sx = pressed ? modelData.x : 0
          root.sy = pressed ? modelData.y : 0
          if (!pressed) root.forceActiveFocus()
        }
      }
    }
  }

  // --- Rotation ---------------------------------------------------------------
  RowLayout {
    Layout.alignment: Qt.AlignHCenter
    spacing: 4
    Button {
      text: "↺ rotate"
      Layout.preferredWidth: 107
      onPressedChanged: { root.sw = pressed ? 1 : 0; if (!pressed) root.forceActiveFocus() }
    }
    Button {
      text: "rotate ↻"
      Layout.preferredWidth: 107
      onPressedChanged: { root.sw = pressed ? -1 : 0; if (!pressed) root.forceActiveFocus() }
    }
  }

  // --- Velocities -------------------------------------------------------------
  Label { text: "Linear velocity: " + linSlider.value.toFixed(2) + " m/s" }
  Slider {
    id: linSlider
    Layout.fillWidth: true
    from: 0.05; to: OmniTeleop.maxLinear; value: OmniTeleop.maxLinear / 2
  }
  Label { text: "Angular velocity: " + angSlider.value.toFixed(2) + " rad/s" }
  Slider {
    id: angSlider
    Layout.fillWidth: true
    from: 0.1; to: OmniTeleop.maxAngular; value: OmniTeleop.maxAngular / 2
  }

  Label {
    Layout.fillWidth: true
    wrapMode: Text.WordWrap
    color: root.activeFocus ? "#2e7d32" : "#757575"
    text: root.activeFocus
          ? "Keyboard active: W/S forward, A/D sideways, Q/E rotate"
          : "Click here to use the keyboard (W/A/S/D, Q/E)"
  }

  MouseArea {
    Layout.fillWidth: true
    Layout.fillHeight: true
    onClicked: root.forceActiveFocus()
  }
}
