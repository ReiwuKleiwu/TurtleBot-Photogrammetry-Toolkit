import QtQuick 2.15
import QtQuick.Controls 2.15

Rectangle {
  id: root
  width: 290
  height: 410
  color: "#152238"
  radius: 8
  border.width: 1
  border.color: "#274472"

  Column {
    anchors.fill: parent
    anchors.margins: 12
    spacing: 8

    Text {
      text: "WASD Camera"
      color: "#f8fafc"
      font.bold: true
      font.pixelSize: 18
    }

    Switch {
      id: enabledSwitch
      text: "Enabled"
      checked: WasdCameraController.active
      onToggled: WasdCameraController.active = checked
    }

    Text {
      text: "W/S move, A/D strafe, Q/E up-down, J/L yaw, I/K pitch, U/O roll, R reset tilt"
      color: "#cbd5e1"
      width: parent.width
      wrapMode: Text.Wrap
      font.pixelSize: 12
    }

    Text {
      text: WasdCameraController.statusText
      color: "#93c5fd"
      width: parent.width
      wrapMode: Text.Wrap
      font.pixelSize: 12
    }

    Text {
      text: "Move Speed: " + WasdCameraController.linearSpeed.toFixed(1)
      color: "#f8fafc"
      font.pixelSize: 12
    }

    Slider {
      width: parent.width
      from: 0.2
      to: 20.0
      value: WasdCameraController.linearSpeed
      onMoved: WasdCameraController.linearSpeed = value
    }

    Text {
      text: "Vertical Speed: " + WasdCameraController.verticalSpeed.toFixed(1)
      color: "#f8fafc"
      font.pixelSize: 12
    }

    Slider {
      width: parent.width
      from: 0.2
      to: 20.0
      value: WasdCameraController.verticalSpeed
      onMoved: WasdCameraController.verticalSpeed = value
    }

    Text {
      text: "Turn Speed: " + WasdCameraController.angularSpeed.toFixed(1)
      color: "#f8fafc"
      font.pixelSize: 12
    }

    Slider {
      width: parent.width
      from: 0.2
      to: 4.0
      value: WasdCameraController.angularSpeed
      onMoved: WasdCameraController.angularSpeed = value
    }
  }
}
