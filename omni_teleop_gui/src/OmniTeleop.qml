import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

// Mantener presionado un boton (o una tecla con el panel enfocado) mueve el
// robot; al soltar se detiene. Se publica a 10 Hz mientras haya un comando.
ColumnLayout {
  id: root
  Layout.minimumWidth: 300
  Layout.minimumHeight: 480
  anchors.fill: parent
  anchors.margins: 8
  spacing: 8
  focus: true

  // Escalas -1, 0, 1 en cada eje segun botones y teclas
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

  onMovingChanged: send()   // envia el primer comando, o el cero al soltar

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

  // --- Pad de 8 direcciones -------------------------------------------------
  // [vx, vy] para cada boton; vy positivo = izquierda (convencion ROS)
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

  // --- Giro -------------------------------------------------------------------
  RowLayout {
    Layout.alignment: Qt.AlignHCenter
    spacing: 4
    Button {
      text: "↺ girar"
      Layout.preferredWidth: 107
      onPressedChanged: { root.sw = pressed ? 1 : 0; if (!pressed) root.forceActiveFocus() }
    }
    Button {
      text: "girar ↻"
      Layout.preferredWidth: 107
      onPressedChanged: { root.sw = pressed ? -1 : 0; if (!pressed) root.forceActiveFocus() }
    }
  }

  // --- Velocidades ------------------------------------------------------------
  Label { text: "Velocidad lineal: " + linSlider.value.toFixed(2) + " m/s" }
  Slider {
    id: linSlider
    Layout.fillWidth: true
    from: 0.05; to: OmniTeleop.maxLinear; value: OmniTeleop.maxLinear / 2
  }
  Label { text: "Velocidad de giro: " + angSlider.value.toFixed(2) + " rad/s" }
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
          ? "Teclado activo: W/S avanzar, A/D lateral, Q/E girar"
          : "Haz clic aqui para usar el teclado (W/A/S/D, Q/E)"
  }

  MouseArea {
    Layout.fillWidth: true
    Layout.fillHeight: true
    onClicked: root.forceActiveFocus()
  }
}
