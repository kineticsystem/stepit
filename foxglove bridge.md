# Connecting a Qt Application to StepIt via foxglove-bridge

This document explains how to connect a remote Qt desktop application to the StepIt robot over WebSocket using foxglove-bridge and the Foxglove WebSocket protocol, without installing ROS on the client machine.

## Prerequisites

**On the robot machine (ROS 2 Jazzy):**

Install foxglove-bridge:

```bash
sudo apt install ros-jazzy-foxglove-bridge
```

**On the Qt client machine:**

Install Qt 6 with the WebSockets module. On Ubuntu:

```bash
sudo apt install qt6-websockets-dev
```

On other Linux distributions or when using the Qt online installer, ensure the `Qt WebSockets` component is selected during installation.

No ROS installation is required on the client machine.

## Overview

foxglove-bridge provides a WebSocket server that exposes ROS 2 topics, services, and parameters using the Foxglove WebSocket protocol. A remote process — such as a Qt desktop application — connects to this server, discovers available topics through server-sent `advertise` frames, and then subscribes to or publishes on those topics.

Compared to rosbridge_suite, foxglove-bridge offers lower latency, native support for ROS 2 QoS settings, and efficient binary CDR encoding. The protocol is more structured than rosbridge's free-form JSON: the server drives topic discovery, and the client works with channel IDs rather than topic name strings in message frames.

The StepIt robot exposes the following interfaces over foxglove-bridge once launched:

| Topic | Type | Purpose |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Read current joint positions and velocities |
| `/velocity_controller/commands` | `std_msgs/Float64MultiArray` | Set joint velocities (rad/s) for joint1–joint5 |
| `/position_controller/commands` | `std_msgs/Float64MultiArray` | Set joint positions (rad) for joint1–joint5 |

## Launching the Robot

On the robot machine, build and launch the robot:

```bash
source install/setup.bash
ros2 launch stepit_description robot.launch.py
```

The launch file starts foxglove-bridge automatically on port 8765. You should see a line similar to the following in the terminal output:

```
[foxglove_bridge]: WebSocket server listening on port 8765
```

The Qt client connects to `ws://<robot-ip>:8765`.

## Foxglove WebSocket Protocol Basics

The Foxglove WebSocket protocol uses a mix of text JSON frames and binary frames. Understanding the distinction is essential before reading the client code.

### Server-to-Client Text Frames

The server sends text frames to convey metadata and control information.

**`serverInfo`** — sent immediately after the client connects. Contains the server name and supported capabilities.

```json
{
  "op": "serverInfo",
  "name": "foxglove_bridge",
  "capabilities": ["clientPublish", "services", "parameters"],
  "supportedEncodings": ["cdr"]
}
```

**`advertise`** — sent whenever new topics become available. Lists topics as channels, each with a unique integer `id` that the client must use to subscribe or publish.

```json
{
  "op": "advertise",
  "channels": [
    {
      "id": 3,
      "topic": "/joint_states",
      "encoding": "cdr",
      "schemaName": "sensor_msgs/msg/JointState",
      "schema": "..."
    },
    {
      "id": 4,
      "topic": "/velocity_controller/commands",
      "encoding": "cdr",
      "schemaName": "std_msgs/msg/Float64MultiArray",
      "schema": "..."
    }
  ]
}
```

**`unadvertise`** — sent when topics are removed. Contains a list of channel IDs that are no longer available.

### Client-to-Server Text Frames

**`subscribe`** — registers the client's interest in one or more channels. Uses channel IDs from the `advertise` frame.

```json
{
  "op": "subscribe",
  "subscriptions": [
    {"id": 1, "channelId": 3}
  ]
}
```

**`unsubscribe`** — cancels a subscription by subscription ID.

```json
{
  "op": "unsubscribe",
  "subscriptionIds": [1]
}
```

**`advertise`** (client-side) — declares that the client will publish on a channel. The client supplies the topic name, encoding, schema name, and schema content. The server replies with a `clientAdvertise` acknowledgement containing the channel ID the client must use when sending binary message frames.

```json
{
  "op": "advertise",
  "channels": [
    {
      "id": 1,
      "topic": "/velocity_controller/commands",
      "encoding": "cdr",
      "schemaName": "std_msgs/msg/Float64MultiArray",
      "schema": "..."
    }
  ]
}
```

**`unadvertise`** (client-side) — cancels a client-side channel.

### Server-to-Client Binary Frames: Message Data

When a subscribed topic receives a message, the server sends a binary frame. The frame layout is:

| Offset | Size | Field |
|---|---|---|
| 0 | 1 byte | Opcode: `0x01` (MessageData) |
| 1 | 4 bytes | Subscription ID (uint32, little-endian) |
| 5 | 8 bytes | Timestamp in nanoseconds (uint64, little-endian) |
| 13 | N bytes | CDR-encoded message payload |

### Client-to-Server Binary Frames: Client Message

When publishing, the client sends a binary frame. The frame layout is:

| Offset | Size | Field |
|---|---|---|
| 0 | 1 byte | Opcode: `0x01` (ClientMessage) |
| 1 | 4 bytes | Client channel ID (uint32, little-endian) |
| 5 | N bytes | CDR-encoded message payload |

### CDR Encoding for StepIt Messages

CDR (Common Data Representation) is the wire format used by ROS 2. All multi-byte integers and doubles are little-endian. Strings and sequences are prefixed with a 4-byte length.

**`std_msgs/Float64MultiArray`** — used for velocity and position commands:

| Bytes | Content |
|---|---|
| 4 | `layout.dim` sequence length: `0` (empty) |
| 4 | `layout.data_offset`: `0` |
| 4 | `data` sequence length: N (number of joints) |
| N × 8 | float64 values, one per joint |

Total for five joints: 20 bytes.

**`sensor_msgs/JointState`** — used for joint state feedback:

| Bytes | Content |
|---|---|
| 4 | `header.stamp.sec` |
| 4 | `header.stamp.nanosec` |
| 4 + L + P | `header.frame_id`: length prefix, UTF-8 chars, padding to 4-byte boundary |
| 4 | `name` sequence length: N |
| (4 + L + P) × N | Each joint name: length prefix, chars, padding |
| 4 | `position` sequence length: N |
| N × 8 | float64 position values |
| 4 | `velocity` sequence length: N |
| N × 8 | float64 velocity values |
| 4 | `effort` sequence length: N |
| N × 8 | float64 effort values |

CDR strings include a null terminator in the length count and are padded to the next 4-byte boundary.

## Connecting from Qt

The class below wraps `QWebSocket` and implements the Foxglove WebSocket protocol. It handles topic discovery, subscriptions, CDR encoding for outgoing messages, and CDR decoding for incoming `sensor_msgs/JointState` messages.

Place `FoxgloveClient.h` and `FoxgloveClient.cpp` in your Qt project.

### FoxgloveClient.h

```cpp
#pragma once

#include <QByteArray>
#include <QHash>
#include <QList>
#include <QObject>
#include <QString>
#include <QWebSocket>

struct JointState
{
    QList<QString> name;
    QList<double>  position;
    QList<double>  velocity;
    QList<double>  effort;
};

class FoxgloveClient : public QObject
{
    Q_OBJECT

public:
    explicit FoxgloveClient(const QString &url, QObject *parent = nullptr);

    void publishVelocity(const QList<double> &velocities);
    void publishPosition(const QList<double> &positions);

signals:
    void jointStatesReceived(JointState state);
    void connected();
    void disconnected();

private slots:
    void onConnected();
    void onTextMessageReceived(const QString &message);
    void onBinaryMessageReceived(const QByteArray &message);
    void onDisconnected();

private:
    // CDR encoding helpers
    static QByteArray encodeCdrFloat64MultiArray(const QList<double> &values);
    static JointState decodeCdrJointState(const QByteArray &payload);

    // Protocol helpers
    void sendTextJson(const QJsonObject &obj);
    void clientAdvertiseTopic(quint32 clientChannelId,
                              const QString &topic,
                              const QString &schemaName,
                              const QString &schema);
    void subscribeToChannel(quint32 subscriptionId, quint32 channelId);
    void sendClientMessage(quint32 clientChannelId, const QByteArray &cdrPayload);

    QWebSocket m_socket;

    // Server channel IDs, populated from "advertise" frames.
    quint32 m_jointStatesChannelId{0};

    // Client-side channel IDs for publishing (chosen by us, not the server).
    static constexpr quint32 k_velocityClientChannelId{1};
    static constexpr quint32 k_positionClientChannelId{2};

    // Subscription IDs (chosen by us).
    static constexpr quint32 k_jointStatesSubscriptionId{1};

    bool m_velocityAdvertised{false};
    bool m_positionAdvertised{false};
};
```

### FoxgloveClient.cpp

```cpp
#include "FoxgloveClient.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QUrl>

#include <cstring>

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

FoxgloveClient::FoxgloveClient(const QString &url, QObject *parent)
    : QObject(parent)
{
    connect(&m_socket, &QWebSocket::connected,
            this, &FoxgloveClient::onConnected);
    connect(&m_socket, &QWebSocket::textMessageReceived,
            this, &FoxgloveClient::onTextMessageReceived);
    connect(&m_socket, &QWebSocket::binaryMessageReceived,
            this, &FoxgloveClient::onBinaryMessageReceived);
    connect(&m_socket, &QWebSocket::disconnected,
            this, &FoxgloveClient::onDisconnected);

    // Request the Foxglove WebSocket sub-protocol.
    QNetworkRequest request{QUrl{url}};
    request.setRawHeader("Sec-WebSocket-Protocol", "foxglove.websocket.v1");
    m_socket.open(request);
}

// ---------------------------------------------------------------------------
// Connection lifecycle
// ---------------------------------------------------------------------------

void FoxgloveClient::onConnected()
{
    // Declare the topics this client will publish on.  The server assigns
    // channel IDs in its response; we use our own client-channel IDs in
    // binary frames.
    const QString multiArraySchema =
        "string[] layout.dim\n"
        "uint32 layout.data_offset\n"
        "float64[] data\n";

    clientAdvertiseTopic(
        k_velocityClientChannelId,
        "/velocity_controller/commands",
        "std_msgs/msg/Float64MultiArray",
        multiArraySchema);

    clientAdvertiseTopic(
        k_positionClientChannelId,
        "/position_controller/commands",
        "std_msgs/msg/Float64MultiArray",
        multiArraySchema);

    m_velocityAdvertised = true;
    m_positionAdvertised = true;

    emit connected();
    // Subscription to /joint_states is deferred until the server sends an
    // "advertise" frame containing the channel ID for that topic.
}

void FoxgloveClient::onDisconnected()
{
    m_jointStatesChannelId = 0;
    m_velocityAdvertised   = false;
    m_positionAdvertised   = false;
    emit disconnected();
}

// ---------------------------------------------------------------------------
// Incoming text frames
// ---------------------------------------------------------------------------

void FoxgloveClient::onTextMessageReceived(const QString &message)
{
    const QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    if (doc.isNull() || !doc.isObject())
        return;

    const QJsonObject frame = doc.object();
    const QString op = frame.value("op").toString();

    if (op == "serverInfo") {
        // Nothing to act on; the server capabilities are in frame["capabilities"].
        return;
    }

    if (op == "advertise") {
        // The server lists available topics.  Find /joint_states and subscribe.
        const QJsonArray channels = frame.value("channels").toArray();
        for (const QJsonValue &ch : channels) {
            const QJsonObject channel = ch.toObject();
            if (channel.value("topic").toString() == "/joint_states") {
                m_jointStatesChannelId =
                    static_cast<quint32>(channel.value("id").toInt());
                subscribeToChannel(k_jointStatesSubscriptionId,
                                   m_jointStatesChannelId);
            }
        }
        return;
    }

    if (op == "unadvertise") {
        const QJsonArray ids = frame.value("channelIds").toArray();
        for (const QJsonValue &v : ids) {
            if (static_cast<quint32>(v.toInt()) == m_jointStatesChannelId)
                m_jointStatesChannelId = 0;
        }
        return;
    }

    // statusMessage and other informational frames are ignored.
}

// ---------------------------------------------------------------------------
// Incoming binary frames — MessageData (opcode 0x01)
// ---------------------------------------------------------------------------

void FoxgloveClient::onBinaryMessageReceived(const QByteArray &message)
{
    // Minimum frame: 1 (opcode) + 4 (subscriptionId) + 8 (timestamp) = 13 bytes.
    if (message.size() < 13)
        return;

    const quint8 opcode = static_cast<quint8>(message[0]);
    if (opcode != 0x01)
        return;  // Only MessageData frames are expected from the server.

    quint32 subscriptionId{};
    std::memcpy(&subscriptionId, message.constData() + 1, 4);

    // Timestamp occupies bytes 5–12; skip it.

    const QByteArray payload = message.mid(13);

    if (subscriptionId == k_jointStatesSubscriptionId) {
        const JointState state = decodeCdrJointState(payload);
        emit jointStatesReceived(state);
    }
}

// ---------------------------------------------------------------------------
// Public publishing API
// ---------------------------------------------------------------------------

void FoxgloveClient::publishVelocity(const QList<double> &velocities)
{
    if (!m_velocityAdvertised)
        return;
    sendClientMessage(k_velocityClientChannelId,
                      encodeCdrFloat64MultiArray(velocities));
}

void FoxgloveClient::publishPosition(const QList<double> &positions)
{
    if (!m_positionAdvertised)
        return;
    sendClientMessage(k_positionClientChannelId,
                      encodeCdrFloat64MultiArray(positions));
}

// ---------------------------------------------------------------------------
// Protocol helpers
// ---------------------------------------------------------------------------

void FoxgloveClient::sendTextJson(const QJsonObject &obj)
{
    const QByteArray data = QJsonDocument(obj).toJson(QJsonDocument::Compact);
    m_socket.sendTextMessage(QString::fromUtf8(data));
}

void FoxgloveClient::clientAdvertiseTopic(quint32 clientChannelId,
                                          const QString &topic,
                                          const QString &schemaName,
                                          const QString &schema)
{
    const QJsonObject channel{
        {"id",         static_cast<int>(clientChannelId)},
        {"topic",      topic},
        {"encoding",   "cdr"},
        {"schemaName", schemaName},
        {"schema",     schema},
    };
    sendTextJson({
        {"op",       "advertise"},
        {"channels", QJsonArray{channel}},
    });
}

void FoxgloveClient::subscribeToChannel(quint32 subscriptionId, quint32 channelId)
{
    const QJsonObject sub{
        {"id",        static_cast<int>(subscriptionId)},
        {"channelId", static_cast<int>(channelId)},
    };
    sendTextJson({
        {"op",            "subscribe"},
        {"subscriptions", QJsonArray{sub}},
    });
}

void FoxgloveClient::sendClientMessage(quint32 clientChannelId,
                                       const QByteArray &cdrPayload)
{
    // Binary frame layout: opcode (1) + channelId (4) + CDR payload (N).
    QByteArray frame;
    frame.reserve(5 + cdrPayload.size());

    frame.append(static_cast<char>(0x01));  // ClientMessage opcode

    char channelIdBytes[4];
    std::memcpy(channelIdBytes, &clientChannelId, 4);
    frame.append(channelIdBytes, 4);

    frame.append(cdrPayload);

    m_socket.sendBinaryMessage(frame);
}

// ---------------------------------------------------------------------------
// CDR encoding: std_msgs/Float64MultiArray
// ---------------------------------------------------------------------------

QByteArray FoxgloveClient::encodeCdrFloat64MultiArray(const QList<double> &values)
{
    // Layout:
    //   layout.dim  sequence (4 bytes, length = 0)
    //   layout.data_offset (4 bytes, value = 0)
    //   data sequence (4 bytes length + N * 8 bytes)
    const quint32 zero  = 0;
    const quint32 count = static_cast<quint32>(values.size());

    QByteArray buf;
    buf.reserve(12 + static_cast<int>(count) * 8);

    // layout.dim: empty sequence
    buf.append(reinterpret_cast<const char *>(&zero), 4);
    // layout.data_offset
    buf.append(reinterpret_cast<const char *>(&zero), 4);
    // data sequence length
    buf.append(reinterpret_cast<const char *>(&count), 4);
    // data values
    for (double v : values)
        buf.append(reinterpret_cast<const char *>(&v), 8);

    return buf;
}

// ---------------------------------------------------------------------------
// CDR decoding: sensor_msgs/JointState
// ---------------------------------------------------------------------------

// Helper: read a CDR string starting at offset.  Advances offset past the
// string content and padding.  Returns an empty string on any error.
static QString readCdrString(const QByteArray &buf, int &offset)
{
    if (offset + 4 > buf.size())
        return {};

    quint32 len{};
    std::memcpy(&len, buf.constData() + offset, 4);
    offset += 4;

    if (len == 0 || offset + static_cast<int>(len) > buf.size())
        return {};

    // len includes the null terminator.
    const QString str = QString::fromUtf8(buf.constData() + offset,
                                          static_cast<int>(len) - 1);
    offset += static_cast<int>(len);

    // CDR strings are padded to the next 4-byte boundary.
    const int rem = offset % 4;
    if (rem != 0)
        offset += 4 - rem;

    return str;
}

JointState FoxgloveClient::decodeCdrJointState(const QByteArray &payload)
{
    JointState state;
    int offset = 0;

    // header.stamp.sec and header.stamp.nanosec
    if (offset + 8 > payload.size())
        return state;
    offset += 8;  // skip timestamp

    // header.frame_id
    readCdrString(payload, offset);

    // name[] sequence
    if (offset + 4 > payload.size())
        return state;
    quint32 nameCount{};
    std::memcpy(&nameCount, payload.constData() + offset, 4);
    offset += 4;

    state.name.reserve(static_cast<int>(nameCount));
    for (quint32 i = 0; i < nameCount; ++i)
        state.name.append(readCdrString(payload, offset));

    // Helper lambda: read a float64 sequence.
    auto readFloat64Sequence = [&](QList<double> &out) {
        if (offset + 4 > payload.size())
            return;
        quint32 count{};
        std::memcpy(&count, payload.constData() + offset, 4);
        offset += 4;

        out.reserve(static_cast<int>(count));
        for (quint32 i = 0; i < count; ++i) {
            if (offset + 8 > payload.size())
                break;
            double v{};
            std::memcpy(&v, payload.constData() + offset, 8);
            out.append(v);
            offset += 8;
        }
    };

    readFloat64Sequence(state.position);
    readFloat64Sequence(state.velocity);
    readFloat64Sequence(state.effort);

    return state;
}
```

### Usage Example

```cpp
// Connect to the robot at 192.168.1.10 on the default foxglove-bridge port.
auto *client = new FoxgloveClient("ws://192.168.1.10:8765", this);

connect(client, &FoxgloveClient::jointStatesReceived,
        this, &MyWidget::onJointStates);

// Set all five joints to 0.5 rad/s.
client->publishVelocity({0.5, 0.5, 0.5, 0.5, 0.5});

// Stop all joints.
client->publishVelocity({0.0, 0.0, 0.0, 0.0, 0.0});

// Move all five joints to target positions (rad).
client->publishPosition({0.0, 1.57, -0.3, 0.8, -1.0});
```

## Subscribing to Joint States

After the WebSocket connects, `onConnected` advertises the publish channels and then waits. When foxglove-bridge sends an `advertise` text frame listing `/joint_states`, `onTextMessageReceived` extracts the channel ID and sends a `subscribe` frame.

From that point, foxglove-bridge sends binary `MessageData` frames each time `/joint_states` is updated. `onBinaryMessageReceived` strips the 13-byte header, calls `decodeCdrJointState`, and emits `jointStatesReceived`.

Parse joint states in a slot:

```cpp
void MyWidget::onJointStates(const JointState &state)
{
    for (int i = 0; i < state.name.size(); ++i) {
        qDebug() << state.name[i]
                 << "position:" << state.position.value(i) << "rad"
                 << "velocity:" << state.velocity.value(i) << "rad/s";
    }
}
```

Do not assume a fixed ordering of joint names in the arrays. Always look up an index from the `name` list rather than using a hardcoded index.

## CMakeLists.txt and .pro Integration

### CMakeLists.txt (Qt 6)

```cmake
cmake_minimum_required(VERSION 3.16)
project(MyRobotUI)

set(CMAKE_CXX_STANDARD 20)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets Network WebSockets)

qt_standard_project_setup()

qt_add_executable(MyRobotUI
    main.cpp
    FoxgloveClient.cpp
    FoxgloveClient.h
    MyWidget.cpp
    MyWidget.h
)

target_link_libraries(MyRobotUI
    PRIVATE
        Qt6::Core
        Qt6::Widgets
        Qt6::Network
        Qt6::WebSockets
)
```

Note that `Qt6::Network` is required in addition to `Qt6::WebSockets` because `FoxgloveClient` uses `QNetworkRequest` to set the `Sec-WebSocket-Protocol` header.

### .pro file (qmake)

```pro
QT += core widgets network websockets

CONFIG += c++20

TARGET = MyRobotUI

SOURCES += \
    main.cpp \
    FoxgloveClient.cpp \
    MyWidget.cpp

HEADERS += \
    FoxgloveClient.h \
    MyWidget.h
```

## Troubleshooting

**WebSocket connection refused**

Verify that the launch file started foxglove-bridge successfully and that port 8765 is reachable from the client machine:

```bash
# On the client machine, replace 192.168.1.10 with the robot's IP address.
nc -zv 192.168.1.10 8765
```

Check that no firewall rule is blocking port 8765 on the robot machine:

```bash
sudo ufw status
```

**`advertise` frame for `/joint_states` is never received**

foxglove-bridge only advertises topics that have at least one active publisher. Confirm that the joint state publisher is running:

```bash
source install/setup.bash
ros2 topic echo /joint_states --once
```

If this command hangs, the controller or the hardware interface is not running. Check the launch output for errors.

**Messages are sent but joints do not move**

Confirm that the target controller is active:

```bash
source install/setup.bash
ros2 control list_controllers
```

The `velocity_controller` and `position_controller` must show `active`. Publishing to a topic whose controller is inactive is silently discarded.

**`jointStatesReceived` signal is never emitted**

The subscription to `/joint_states` is sent only after `onConnected` fires and after the server's `advertise` frame arrives. If you call `publishVelocity` before the `connected` signal is emitted, `m_velocityAdvertised` will be false and the message will be dropped. Wait for the `connected` signal before publishing.

**CDR decoding produces wrong values**

foxglove-bridge sends CDR payloads with a 4-byte encapsulation header before the message data on some configurations. If joint names or positions appear garbled, skip the first 4 bytes of the payload in `decodeCdrJointState` by initializing `offset = 4` instead of `offset = 0`. Check the actual byte content with:

```cpp
qDebug() << payload.toHex(' ');
```

A CDR encapsulation header looks like `00 01 00 00` (little-endian CDR, no padding).

**Sub-protocol negotiation fails**

foxglove-bridge requires the WebSocket sub-protocol `foxglove.websocket.v1`. If the server closes the connection immediately after the handshake, confirm that the `Sec-WebSocket-Protocol` header is being set correctly. Some network proxies strip custom WebSocket headers; connect directly to the robot to rule this out.
