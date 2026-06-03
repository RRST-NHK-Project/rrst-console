import React, { useEffect, useRef, useState } from "react";
import * as ROSLIB from "roslib";
import "./App.css";
import appPackage from "../package.json";

const GUI_VERSION = process.env.REACT_APP_UI_VERSION || appPackage.version || "0.0.0";

const SETTINGS_STORAGE_KEY = "soki_console_settings_v1";
const TARGET_PRESETS_STORAGE_KEY = "soki_console_target_presets_v1";
const LOG_LIMIT = 80;
const MONITOR_LIMIT = 30;

const PAGE_DEFINITIONS = [
    { key: "overview", label: "概要" },
    { key: "control", label: "座標制御" },
    { key: "automation", label: "自動化" },
    { key: "actuator", label: "アクチュエータ" },
    { key: "esc", label: "ESC設定" },
    { key: "debug", label: "デバッグ" },
    { key: "settings", label: "設定" },
];

const STATE_DEFINITIONS = [
    { key: "BOOT", label: "起動" },
    { key: "INIT_ENCODERS", label: "初期化" },
    { key: "STANDBY", label: "待機" },
    { key: "MANUAL", label: "手動" },
    { key: "AUTO_MOVE", label: "自動移動" },
    { key: "HOLD", label: "姿勢保持" },
    { key: "PICK_READY", label: "把持準備" },
    { key: "PLACE_READY", label: "受け渡し" },
    { key: "DEBUG", label: "デバッグ" },
    { key: "FAULT", label: "Fault" },
    { key: "ESTOP", label: "非常停止" },
];

const TOPIC_FIELD_DEFINITIONS = [
    { key: "jointFeedback", label: "関節FB" },
    { key: "encoderRaw", label: "エンコーダ生値" },
    { key: "encoderStatus", label: "エンコーダ初期化状態" },
    { key: "motorFeedback", label: "モータFB" },
    { key: "stateFeedback", label: "状態FB" },
    { key: "faultFeedback", label: "Fault FB" },
    { key: "motionFeedback", label: "移動中FB" },
    { key: "targetCommand", label: "座標指令" },
    { key: "stateCommand", label: "状態指令" },
    { key: "encoderInitCommand", label: "初期化指令" },
    { key: "estopCommand", label: "非常停止指令" },
    { key: "motorDebugCommand", label: "デバッグ出力" },
    { key: "gripperCommand", label: "ハンド指令" },
    { key: "escCtrlCommand", label: "ESC cmd" },
    { key: "escCtrlParams", label: "ESC params" },
    { key: "escSerialRx", label: "ESC serial rx" },
];

const DEFAULT_TOPIC_CONFIG = {
    jointFeedback: { name: "/soki/joint_feedback", type: "std_msgs/msg/Float32MultiArray" },
    encoderRaw: { name: "/soki/encoder_raw", type: "std_msgs/msg/Int32MultiArray" },
    encoderStatus: { name: "/soki/encoder_ready", type: "std_msgs/msg/Int32MultiArray" },
    motorFeedback: { name: "/soki/motor_feedback", type: "std_msgs/msg/Float32MultiArray" },
    stateFeedback: { name: "/soki/state", type: "std_msgs/msg/String" },
    faultFeedback: { name: "/soki/fault", type: "std_msgs/msg/String" },
    motionFeedback: { name: "/soki/is_moving", type: "std_msgs/msg/Bool" },
    targetCommand: { name: "/soki/cmd/target", type: "std_msgs/msg/Float32MultiArray" },
    stateCommand: { name: "/soki/cmd/state", type: "std_msgs/msg/String" },
    encoderInitCommand: { name: "/soki/cmd/init_encoders", type: "std_msgs/msg/Bool" },
    estopCommand: { name: "/soki/cmd/estop", type: "std_msgs/msg/Bool" },
    motorDebugCommand: { name: "/soki/cmd/motor_debug", type: "std_msgs/msg/Float32MultiArray" },
    gripperCommand: { name: "/soki/cmd/gripper", type: "std_msgs/msg/Float32MultiArray" },

    escCtrlCommand: { name: "/esc_ctrl/cmd", type: "std_msgs/msg/Float32MultiArray" },
    escCtrlParams: { name: "/esc_ctrl/cmd/params", type: "std_msgs/msg/Float32MultiArray" },
    escSerialRx: { name: "/serial_rx_153", type: "std_msgs/msg/Int16MultiArray" },
};

const AXIS_DEFINITIONS = [
    { key: "r", label: "r", unit: "m", stepKey: "r" },
    { key: "thetaDeg", label: "theta", unit: "deg", stepKey: "theta" },
    { key: "z", label: "z", unit: "m", stepKey: "z" },
    { key: "wristDeg", label: "ハンド姿勢", unit: "deg", stepKey: "wrist" },
    { key: "gripper", label: "ハンド開閉", unit: "arb", stepKey: "gripper" },
];

const DEFAULT_STEPS = {
    r: "0.01",
    theta: "5",
    z: "0.01",
    wrist: "5",
    gripper: "10",
};

const DEFAULT_TOLERANCES = {
    r: "0.005",
    theta: "2.0",
    z: "0.005",
    wrist: "3.0",
};

const readStoredJson = (key, fallback) => {
    try {
        const raw = window.localStorage.getItem(key);
        if (!raw) {
            return fallback;
        }
        return JSON.parse(raw);
    } catch (_error) {
        return fallback;
    }
};

const writeStoredJson = (key, value) => {
    try {
        window.localStorage.setItem(key, JSON.stringify(value));
    } catch (_error) {
    }
};

const parseNumber = (value, fallback = 0) => {
    const parsed = Number.parseFloat(String(value));
    return Number.isFinite(parsed) ? parsed : fallback;
};

const parseInteger = (value, fallback = 0) => {
    const parsed = Number.parseInt(String(value), 10);
    return Number.isFinite(parsed) ? parsed : fallback;
};

const radToDeg = (value) => value * 180 / Math.PI;
const degToRad = (value) => value * Math.PI / 180;
const formatFixed = (value, digits = 3) => parseNumber(value).toFixed(digits);

const createEmptyJointFeedback = () => ({
    r: 0,
    thetaDeg: 0,
    z: 0,
    wristDeg: 0,
    gripper: 0,
});

const createEmptyMotorFeedback = () => ({
    theta: 0,
    diffA: 0,
    diffB: 0,
    wrist: 0,
    gripper: 0,
});

const createEmptyEncoderRaw = () => ({
    r: 0,
    theta: 0,
    z: 0,
    wrist: 0,
});

const createEmptyEncoderReady = () => ({
    r: false,
    theta: false,
    z: false,
    wrist: false,
});

const createEmptyEscFeedback = () => ({
    angleDeg: 0,
    velocityRadS: 0,
    target: 0,
    mode: 0,
    voltageLimitV: 0,
    rpm: 0,
    velocityLimitRadS: null,
    currentLimitA: null,
    velocityPid: null,
    velocityOutputRamp: null,
    velocityLpfTfS: null,
    anglePGain: null,
});

const normalizeJointFeedback = (msg) => {
    const data = Array.isArray(msg?.data) ? msg.data : [];
    return {
        r: parseNumber(data[0]),
        thetaDeg: radToDeg(parseNumber(data[1])),
        z: parseNumber(data[2]),
        wristDeg: radToDeg(parseNumber(data[3])),
        gripper: parseNumber(data[4]),
    };
};

const normalizeEncoderRaw = (msg) => {
    const data = Array.isArray(msg?.data) ? msg.data : [];
    return {
        r: parseInteger(data[0]),
        theta: parseInteger(data[1]),
        z: parseInteger(data[2]),
        wrist: parseInteger(data[3]),
    };
};

const normalizeEncoderReady = (msg) => {
    const data = Array.isArray(msg?.data) ? msg.data : [];
    return {
        r: Boolean(parseInteger(data[0])),
        theta: Boolean(parseInteger(data[1])),
        z: Boolean(parseInteger(data[2])),
        wrist: Boolean(parseInteger(data[3])),
    };
};

const normalizeMotorFeedback = (msg) => {
    const data = Array.isArray(msg?.data) ? msg.data : [];
    return {
        theta: parseNumber(data[0]),
        diffA: parseNumber(data[1]),
        diffB: parseNumber(data[2]),
        wrist: parseNumber(data[3]),
        gripper: parseNumber(data[4]),
    };
};

const normalizeEscSerialRx = (msg) => {
    const data = Array.isArray(msg?.data) ? msg.data : [];

    const mode = parseInteger(data[4]);
    const angleDeg = parseInteger(data[1]) * 0.1;
    const velocityRadS = parseInteger(data[2]) * 0.1;
    const target = parseInteger(data[3]) * 0.1;
    const voltageLimitV = parseInteger(data[5]) * 0.1;
    const rpm = parseInteger(data[6]);

    const hasTuningEcho = data.length >= 15;
    return {
        angleDeg,
        velocityRadS,
        target,
        mode,
        voltageLimitV,
        rpm,
        velocityLimitRadS: hasTuningEcho ? parseInteger(data[7]) * 0.1 : null,
        currentLimitA: hasTuningEcho ? parseInteger(data[8]) * 0.1 : null,
        velocityPid: hasTuningEcho
            ? {
                p: parseInteger(data[9]) * 0.001,
                i: parseInteger(data[10]) * 0.001,
                d: parseInteger(data[11]) * 0.001,
            }
            : null,
        velocityOutputRamp: hasTuningEcho ? parseInteger(data[12]) : null,
        velocityLpfTfS: hasTuningEcho ? parseInteger(data[13]) / 1000.0 : null,
        anglePGain: hasTuningEcho ? parseInteger(data[14]) * 0.01 : null,
    };
};

const createLogEntry = (kind, message) => ({
    id: `${Date.now()}-${Math.random().toString(16).slice(2)}`,
    time: new Date().toLocaleTimeString("ja-JP"),
    kind,
    message,
});

const buildTargetPayload = (targetInputs, holdHandOrientation) => {
    const r = parseNumber(targetInputs.r);
    const thetaDeg = parseNumber(targetInputs.theta);
    const z = parseNumber(targetInputs.z);
    const handInputDeg = parseNumber(targetInputs.hand);
    const gripper = parseNumber(targetInputs.gripper);
    const wristLocalDeg = holdHandOrientation ? handInputDeg - thetaDeg : handInputDeg;

    return {
        display: {
            r,
            thetaDeg,
            z,
            handInputDeg,
            wristLocalDeg,
            handWorldDeg: holdHandOrientation ? handInputDeg : thetaDeg + handInputDeg,
            gripper,
        },
        data: [r, degToRad(thetaDeg), z, degToRad(wristLocalDeg), gripper],
    };
};

const buildPresetSummary = (presetInputs, holdHandOrientation) => {
    const payload = buildTargetPayload(presetInputs, holdHandOrientation);
    return `r ${payload.display.r.toFixed(3)} / th ${payload.display.thetaDeg.toFixed(1)} / z ${payload.display.z.toFixed(3)}`;
};

const buildDifferentialPreview = (rValue, zValue) => ({
    diffA: zValue + rValue,
    diffB: zValue - rValue,
});

const getStatusClassName = (connected, hasFault, estop) => {
    if (estop || hasFault) {
        return "status-bad";
    }
    return connected ? "status-ok" : "status-pending";
};

const AxisStatusCard = ({
    label,
    unit,
    currentValue,
    targetValue,
    errorValue,
    rawValue,
    ready,
    digits,
}) => (
    <div className="serial-packet-section soki-axis-card">
        <div className="soki-axis-header">
            <h3>{label}</h3>
            <span className={`status-pill ${ready ? "status-ok" : "status-pending"}`}>
                {ready ? "Ready" : "未初期化"}
            </span>
        </div>
        <div className="soki-kv-grid">
            <span>現在値</span>
            <strong>{formatFixed(currentValue, digits)} {unit}</strong>
            <span>目標値</span>
            <strong>{formatFixed(targetValue, digits)} {unit}</strong>
            <span>偏差</span>
            <strong>{formatFixed(errorValue, digits)} {unit}</strong>
            <span>Encoder</span>
            <strong>{rawValue}</strong>
        </div>
    </div>
);

function App() {
    const defaultRosHost = window.location.hostname || "localhost";
    const initialSettingsRef = useRef(
        readStoredJson(SETTINGS_STORAGE_KEY, {
            rosHost: defaultRosHost,
            rosPort: "9090",
            autoInitOnConnect: true,
            multiTabMode: true,
            topicConfig: DEFAULT_TOPIC_CONFIG,
            targetSteps: DEFAULT_STEPS,
            targetTolerances: DEFAULT_TOLERANCES,
            topicMonitorName: "/rosout",
            topicMonitorType: "rcl_interfaces/msg/Log",
        })
    );
    const initialSettings = initialSettingsRef.current;

    const [rosHostInput, setRosHostInput] = useState(initialSettings.rosHost || defaultRosHost);
    const [rosPortInput, setRosPortInput] = useState(initialSettings.rosPort || "9090");
    const [rosEndpoint, setRosEndpoint] = useState({
        host: initialSettings.rosHost || defaultRosHost,
        port: initialSettings.rosPort || "9090",
    });
    const [status, setStatus] = useState(process.env.NODE_ENV === "test" ? "接続待機" : "接続中...");
    const [rosConnected, setRosConnected] = useState(false);
    const [operationArmed, setOperationArmed] = useState(false);
    const [autoInitOnConnect, setAutoInitOnConnect] = useState(Boolean(initialSettings.autoInitOnConnect));
    const [multiTabMode, setMultiTabMode] = useState(Boolean(initialSettings.multiTabMode));
    const [activePages, setActivePages] = useState(["overview", "control", "automation"]);
    const [topicConfig, setTopicConfig] = useState({
        ...DEFAULT_TOPIC_CONFIG,
        ...(initialSettings.topicConfig || {}),
    });
    const [targetSteps, setTargetSteps] = useState({
        ...DEFAULT_STEPS,
        ...(initialSettings.targetSteps || {}),
    });
    const [targetTolerances, setTargetTolerances] = useState({
        ...DEFAULT_TOLERANCES,
        ...(initialSettings.targetTolerances || {}),
    });

    const [jointFeedback, setJointFeedback] = useState(createEmptyJointFeedback());
    const [encoderRaw, setEncoderRaw] = useState(createEmptyEncoderRaw());
    const [encoderReady, setEncoderReady] = useState(createEmptyEncoderReady());
    const [motorFeedback, setMotorFeedback] = useState(createEmptyMotorFeedback());
    const [feedbackUpdatedAt, setFeedbackUpdatedAt] = useState("");
    const [motorUpdatedAt, setMotorUpdatedAt] = useState("");
    const [systemState, setSystemState] = useState("BOOT");
    const [stateReason, setStateReason] = useState("GUI起動");
    const [faultText, setFaultText] = useState("");
    const [isMoving, setIsMoving] = useState(false);
    const [motionInfo, setMotionInfo] = useState("未送信");
    const [actuatorInfo, setActuatorInfo] = useState("未送信");
    const [settingsInfo, setSettingsInfo] = useState("接続設定を確認してください");
    const [topicMonitorInfo, setTopicMonitorInfo] = useState("未開始");
    const [logs, setLogs] = useState([]);
    const [topicMonitorName, setTopicMonitorName] = useState(initialSettings.topicMonitorName || "/rosout");
    const [topicMonitorType, setTopicMonitorType] = useState(initialSettings.topicMonitorType || "rcl_interfaces/msg/Log");
    const [topicMonitorRunning, setTopicMonitorRunning] = useState(false);
    const [topicMonitorMessages, setTopicMonitorMessages] = useState([]);
    const [debugSnapshots, setDebugSnapshots] = useState({
        jointFeedback: "",
        encoderRaw: "",
        encoderStatus: "",
        motorFeedback: "",
        stateFeedback: "",
        faultFeedback: "",
        motionFeedback: "",
        escSerialRx: "",
    });

    const [targetInputs, setTargetInputs] = useState({
        r: formatFixed(jointFeedback.r, 3),
        theta: formatFixed(jointFeedback.thetaDeg, 1),
        z: formatFixed(jointFeedback.z, 3),
        hand: formatFixed(jointFeedback.thetaDeg + jointFeedback.wristDeg, 1),
        gripper: formatFixed(jointFeedback.gripper, 1),
    });
    const [holdHandOrientation, setHoldHandOrientation] = useState(true);
    const [presetNameInput, setPresetNameInput] = useState("受け取り位置");
    const [savedTargets, setSavedTargets] = useState(
        readStoredJson(TARGET_PRESETS_STORAGE_KEY, [])
    );
    const [motorCommandInputs, setMotorCommandInputs] = useState({
        theta: "0",
        diffA: "0",
        diffB: "0",
        wrist: "0",
        gripper: "0",
    });

    const [escCommandInputs, setEscCommandInputs] = useState({
        enable: "1",
        mode: "0",
        target: "0",
        voltageLimit: "6.0",
    });

    const [escParamInputs, setEscParamInputs] = useState({
        velocityLimit: "1500",
        currentLimit: "10",
        velP: "0.02",
        velI: "0.0",
        velD: "0.0",
        velRamp: "1000",
        velLpfTf: "0.02",
        angleP: "8.0",
    });

    const [escFeedback, setEscFeedback] = useState(createEmptyEscFeedback());
    const [escInfo, setEscInfo] = useState("未送信");

    const rosRef = useRef(null);
    const publishersRef = useRef({});
    const subscriptionsRef = useRef([]);
    const topicMonitorSubRef = useRef(null);
    const operationArmedRef = useRef(operationArmed);
    const startupInitRequestedRef = useRef(false);
    const transitionStateRef = useRef(null);
    const requestEncoderInitializationRef = useRef(null);

    operationArmedRef.current = operationArmed;

    const appendLog = (kind, message) => {
        setLogs((prev) => [createLogEntry(kind, message), ...prev].slice(0, LOG_LIMIT));
    };

    const updateSnapshot = (key, msg) => {
        setDebugSnapshots((prev) => ({
            ...prev,
            [key]: JSON.stringify(msg, null, 2),
        }));
    };

    const publishMessage = (publisherKey, message, options = {}) => {
        const { requireArm = false, failureText = "送信に失敗しました", successText = "送信しました", infoSetter = null } = options;
        if (requireArm && !operationArmedRef.current) {
            if (infoSetter) {
                infoSetter("操作ロックが有効のため送信できません");
            }
            appendLog("warn", `BLOCKED ${publisherKey}: operation lock enabled`);
            return false;
        }

        const publisher = publishersRef.current[publisherKey];
        if (!publisher) {
            if (infoSetter) {
                infoSetter("ROS未接続のため送信できません");
            }
            appendLog("warn", `BLOCKED ${publisherKey}: publisher unavailable`);
            return false;
        }

        try {
            publisher.publish(message);
            if (infoSetter) {
                infoSetter(successText);
            }
            appendLog("tx", `${publisherKey} ${JSON.stringify(message)}`);
            return true;
        } catch (_error) {
            if (infoSetter) {
                infoSetter(failureText);
            }
            appendLog("error", `${publisherKey} publish failed`);
            return false;
        }
    };

    const transitionState = (nextState, reason, publishState = true) => {
        setSystemState(nextState);
        setStateReason(reason);
        appendLog("state", `${nextState}: ${reason}`);
        if (publishState) {
            publishMessage(
                "stateCommand",
                { data: nextState },
                {
                    requireArm: false,
                    infoSetter: setMotionInfo,
                    successText: `状態を ${nextState} に更新しました`,
                    failureText: "状態更新に失敗しました",
                }
            );
        }
    };

    const requestEncoderInitialization = (sourceText) => {
        const published = publishMessage(
            "encoderInitCommand",
            { data: true },
            {
                requireArm: false,
                infoSetter: setMotionInfo,
                successText: "エンコーダ初期化要求を送信しました",
                failureText: "エンコーダ初期化要求の送信に失敗しました",
            }
        );
        if (published) {
            transitionState("INIT_ENCODERS", sourceText, true);
        }
    };

    transitionStateRef.current = transitionState;
    requestEncoderInitializationRef.current = requestEncoderInitialization;

    const applyConnectionSettings = () => {
        const host = rosHostInput.trim() || defaultRosHost;
        const port = rosPortInput.trim() || "9090";
        setStatus("接続中...");
        setSettingsInfo(`接続先を ${host}:${port} に更新中です`);
        setRosEndpoint({ host, port });
    };

    const handlePageToggle = (pageKey) => {
        if (!multiTabMode) {
            setActivePages([pageKey]);
            return;
        }

        setActivePages((prev) => {
            if (prev.includes(pageKey)) {
                return prev.length === 1 ? prev : prev.filter((key) => key !== pageKey);
            }
            return [...prev, pageKey];
        });
    };

    const updateTargetInput = (key, value) => {
        setTargetInputs((prev) => ({
            ...prev,
            [key]: value,
        }));
    };

    const updateTargetStep = (key, value) => {
        setTargetSteps((prev) => ({
            ...prev,
            [key]: value,
        }));
    };

    const updateTolerance = (key, value) => {
        setTargetTolerances((prev) => ({
            ...prev,
            [key]: value,
        }));
    };

    const nudgeTarget = (key, direction) => {
        const current = parseNumber(targetInputs[key]);
        const stepValue = parseNumber(targetSteps[key], 0);
        updateTargetInput(key, String(current + stepValue * direction));
    };

    const copyCurrentToTarget = () => {
        setTargetInputs({
            r: formatFixed(jointFeedback.r, 3),
            theta: formatFixed(jointFeedback.thetaDeg, 1),
            z: formatFixed(jointFeedback.z, 3),
            hand: formatFixed(jointFeedback.thetaDeg + jointFeedback.wristDeg, 1),
            gripper: formatFixed(jointFeedback.gripper, 1),
        });
        setMotionInfo("現在フィードバックを目標値へ反映しました");
    };

    const sendTargetCommand = (transitionToAutoMove) => {
        const payload = buildTargetPayload(targetInputs, holdHandOrientation);
        const sent = publishMessage(
            "targetCommand",
            { data: payload.data },
            {
                requireArm: true,
                infoSetter: setMotionInfo,
                successText: `目標送信: r=${payload.display.r.toFixed(3)} theta=${payload.display.thetaDeg.toFixed(1)} z=${payload.display.z.toFixed(3)}`,
                failureText: "目標送信に失敗しました",
            }
        );

        if (sent && transitionToAutoMove) {
            transitionState("AUTO_MOVE", "指定座標への自動移動を開始", true);
        }
    };

    const saveTargetPreset = () => {
        const label = presetNameInput.trim() || `Preset ${savedTargets.length + 1}`;
        const nextPreset = {
            id: `${Date.now()}-${Math.random().toString(16).slice(2)}`,
            label,
            holdHandOrientation,
            inputs: { ...targetInputs },
            createdAt: new Date().toLocaleString("ja-JP"),
            summary: buildPresetSummary(targetInputs, holdHandOrientation),
        };
        setSavedTargets((prev) => [nextPreset, ...prev].slice(0, 16));
        setMotionInfo(`プリセット "${label}" を保存しました`);
    };

    const sendPresetTarget = (preset) => {
        const payload = buildTargetPayload(preset.inputs, Boolean(preset.holdHandOrientation));
        const sent = publishMessage(
            "targetCommand",
            { data: payload.data },
            {
                requireArm: true,
                infoSetter: setMotionInfo,
                successText: `プリセット送信: ${preset.label}`,
                failureText: "プリセット送信に失敗しました",
            }
        );
        if (sent) {
            transitionState("AUTO_MOVE", `プリセット ${preset.label} へ移動`, true);
        }
    };

    const applyPreset = (preset, sendImmediately) => {
        setTargetInputs({ ...preset.inputs });
        setHoldHandOrientation(Boolean(preset.holdHandOrientation));
        setPresetNameInput(preset.label);
        setMotionInfo(`プリセット "${preset.label}" を復元しました`);
        if (sendImmediately) {
            sendPresetTarget(preset);
        }
    };

    const deletePreset = (presetId) => {
        setSavedTargets((prev) => prev.filter((preset) => preset.id !== presetId));
    };

    const clearMotorCommands = () => {
        setMotorCommandInputs({ theta: "0", diffA: "0", diffB: "0", wrist: "0", gripper: "0" });
        setActuatorInfo("デバッグ出力をゼロクリアしました");
    };

    const sendMotorDebugCommand = () => {
        const payload = [
            parseNumber(motorCommandInputs.theta),
            parseNumber(motorCommandInputs.diffA),
            parseNumber(motorCommandInputs.diffB),
            parseNumber(motorCommandInputs.wrist),
            parseNumber(motorCommandInputs.gripper),
        ];
        publishMessage(
            "motorDebugCommand",
            { data: payload },
            {
                requireArm: true,
                infoSetter: setActuatorInfo,
                successText: "モータデバッグ指令を送信しました",
                failureText: "モータデバッグ指令の送信に失敗しました",
            }
        );
    };

    const sendGripperCommand = () => {
        const payload = [parseNumber(targetInputs.gripper)];
        publishMessage(
            "gripperCommand",
            { data: payload },
            {
                requireArm: true,
                infoSetter: setActuatorInfo,
                successText: "ハンド指令を送信しました",
                failureText: "ハンド指令の送信に失敗しました",
            }
        );
    };

    const sendEmergencyStop = () => {
        publishMessage(
            "estopCommand",
            { data: true },
            {
                requireArm: false,
                infoSetter: setMotionInfo,
                successText: "非常停止を送信しました",
                failureText: "非常停止の送信に失敗しました",
            }
        );
        setOperationArmed(false);
        transitionState("ESTOP", "GUIから非常停止を実行", false);
    };

    const sendEscCommand = () => {
        const payload = [
            parseNumber(escCommandInputs.enable) ? 1 : 0,
            parseNumber(escCommandInputs.mode) ? 1 : 0,
            parseNumber(escCommandInputs.target),
            parseNumber(escCommandInputs.voltageLimit),
        ];

        publishMessage(
            "escCtrlCommand",
            { data: payload },
            {
                requireArm: true,
                infoSetter: setEscInfo,
                successText: "ESCコマンドを送信しました",
                failureText: "ESCコマンドの送信に失敗しました",
            }
        );
    };

    const sendEscParams = () => {
        const payload = [
            parseNumber(escCommandInputs.voltageLimit),
            parseNumber(escParamInputs.velocityLimit),
            parseNumber(escParamInputs.currentLimit),
            parseNumber(escParamInputs.velP),
            parseNumber(escParamInputs.velI),
            parseNumber(escParamInputs.velD),
            parseNumber(escParamInputs.velRamp),
            parseNumber(escParamInputs.velLpfTf),
            parseNumber(escParamInputs.angleP),
        ];

        publishMessage(
            "escCtrlParams",
            { data: payload },
            {
                requireArm: true,
                infoSetter: setEscInfo,
                successText: "ESCパラメータを送信しました",
                failureText: "ESCパラメータの送信に失敗しました",
            }
        );
    };

    useEffect(() => {
        writeStoredJson(SETTINGS_STORAGE_KEY, {
            rosHost: rosHostInput,
            rosPort: rosPortInput,
            autoInitOnConnect,
            multiTabMode,
            topicConfig,
            targetSteps,
            targetTolerances,
            topicMonitorName,
            topicMonitorType,
        });
    }, [rosHostInput, rosPortInput, autoInitOnConnect, multiTabMode, topicConfig, targetSteps, targetTolerances, topicMonitorName, topicMonitorType]);

    useEffect(() => {
        writeStoredJson(TARGET_PRESETS_STORAGE_KEY, savedTargets);
    }, [savedTargets]);

    useEffect(() => {
        if (process.env.NODE_ENV === "test") {
            return undefined;
        }

        let isDisposed = false;
        subscriptionsRef.current = [];
        publishersRef.current = {};

        const wsScheme = window.location.protocol === "https:" ? "wss" : "ws";
        const rosUrl = `${wsScheme}://${rosEndpoint.host}:${rosEndpoint.port}`;
        setStatus("接続中...");
        setRosConnected(false);

        let ros;
        try {
            ros = new ROSLIB.Ros({ url: rosUrl });
            rosRef.current = ros;
        } catch (_error) {
            setStatus("エラー");
            setRosConnected(false);
            setSettingsInfo(`rosbridge に接続できません: ${rosUrl}`);
            return undefined;
        }

        const cleanupSubscriptions = () => {
            subscriptionsRef.current.forEach((topic) => {
                try {
                    topic.unsubscribe();
                } catch (_error) {
                }
            });
            subscriptionsRef.current = [];
            if (topicMonitorSubRef.current) {
                try {
                    topicMonitorSubRef.current.unsubscribe();
                } catch (_error) {
                }
                topicMonitorSubRef.current = null;
            }
            publishersRef.current = {};
        };

        const registerPublisher = (key) => {
            const config = topicConfig[key];
            publishersRef.current[key] = new ROSLIB.Topic({
                ros,
                name: config.name,
                messageType: config.type,
            });
        };

        const registerSubscription = (key, handler) => {
            const config = topicConfig[key];
            const topic = new ROSLIB.Topic({
                ros,
                name: config.name,
                messageType: config.type,
            });
            topic.subscribe((msg) => {
                if (isDisposed) {
                    return;
                }
                handler(msg);
            });
            subscriptionsRef.current.push(topic);
        };

        ros.on("connection", () => {
            if (isDisposed) {
                return;
            }

            setStatus("接続OK");
            setRosConnected(true);
            setSettingsInfo(`接続中: ${rosUrl}`);
            appendLog("info", `ROS connected: ${rosUrl}`);

            [
                "targetCommand",
                "stateCommand",
                "encoderInitCommand",
                "estopCommand",
                "motorDebugCommand",
                "gripperCommand",
                "escCtrlCommand",
                "escCtrlParams",
            ].forEach(registerPublisher);

            registerSubscription("jointFeedback", (msg) => {
                const nextValue = normalizeJointFeedback(msg);
                setJointFeedback(nextValue);
                setFeedbackUpdatedAt(new Date().toLocaleTimeString("ja-JP"));
                updateSnapshot("jointFeedback", msg);
            });

            registerSubscription("encoderRaw", (msg) => {
                setEncoderRaw(normalizeEncoderRaw(msg));
                updateSnapshot("encoderRaw", msg);
            });

            registerSubscription("encoderStatus", (msg) => {
                setEncoderReady(normalizeEncoderReady(msg));
                updateSnapshot("encoderStatus", msg);
            });

            registerSubscription("motorFeedback", (msg) => {
                setMotorFeedback(normalizeMotorFeedback(msg));
                setMotorUpdatedAt(new Date().toLocaleTimeString("ja-JP"));
                updateSnapshot("motorFeedback", msg);
            });

            registerSubscription("stateFeedback", (msg) => {
                const nextState = String(msg?.data || "").trim();
                if (nextState) {
                    setSystemState(nextState);
                }
                updateSnapshot("stateFeedback", msg);
            });

            registerSubscription("faultFeedback", (msg) => {
                const nextFault = String(msg?.data || "").trim();
                setFaultText(nextFault);
                if (nextFault) {
                    appendLog("fault", nextFault);
                }
                updateSnapshot("faultFeedback", msg);
            });

            registerSubscription("motionFeedback", (msg) => {
                setIsMoving(Boolean(msg?.data));
                updateSnapshot("motionFeedback", msg);
            });

            registerSubscription("escSerialRx", (msg) => {
                setEscFeedback(normalizeEscSerialRx(msg));
                updateSnapshot("escSerialRx", msg);
            });
        });

        ros.on("error", () => {
            if (isDisposed) {
                return;
            }
            setStatus("エラー");
            setRosConnected(false);
            setSettingsInfo(`接続エラー: ${rosUrl}`);
        });

        ros.on("close", () => {
            if (isDisposed) {
                return;
            }
            setStatus("切断");
            setRosConnected(false);
            setSettingsInfo("rosbridge との接続が切断されました");
            startupInitRequestedRef.current = false;
            cleanupSubscriptions();
        });

        return () => {
            isDisposed = true;
            cleanupSubscriptions();
            try {
                ros.close();
            } catch (_error) {
            }
            rosRef.current = null;
        };
    }, [rosEndpoint, topicConfig]);

    useEffect(() => {
        if (!rosConnected || !autoInitOnConnect || startupInitRequestedRef.current) {
            return;
        }
        startupInitRequestedRef.current = true;
        requestEncoderInitializationRef.current?.("接続時自動初期化");
    }, [rosConnected, autoInitOnConnect]);

    useEffect(() => {
        if (!rosConnected) {
            startupInitRequestedRef.current = false;
        }
    }, [rosConnected]);

    const encoderInitialized = encoderReady.r && encoderReady.theta && encoderReady.z && encoderReady.wrist;

    const targetPayload = buildTargetPayload(targetInputs, holdHandOrientation);
    const targetErrors = {
        r: parseNumber(targetInputs.r) - jointFeedback.r,
        thetaDeg: parseNumber(targetInputs.theta) - jointFeedback.thetaDeg,
        z: parseNumber(targetInputs.z) - jointFeedback.z,
        wristDeg: targetPayload.display.wristLocalDeg - jointFeedback.wristDeg,
    };

    const targetReached =
        Math.abs(targetErrors.r) <= parseNumber(targetTolerances.r, 0.005) &&
        Math.abs(targetErrors.thetaDeg) <= parseNumber(targetTolerances.theta, 2.0) &&
        Math.abs(targetErrors.z) <= parseNumber(targetTolerances.z, 0.005) &&
        Math.abs(targetErrors.wristDeg) <= parseNumber(targetTolerances.wrist, 3.0);

    useEffect(() => {
        if (systemState !== "AUTO_MOVE" || !encoderInitialized || !targetReached) {
            return;
        }
        transitionStateRef.current?.("HOLD", "フィードバック上で目標到達を確認", true);
        setMotionInfo("目標到達を検出し HOLD へ遷移しました");
    }, [systemState, encoderInitialized, targetReached]);

    useEffect(() => {
        if (!rosConnected || !topicMonitorRunning || !rosRef.current) {
            if (topicMonitorSubRef.current) {
                try {
                    topicMonitorSubRef.current.unsubscribe();
                } catch (_error) {
                }
                topicMonitorSubRef.current = null;
            }
            return undefined;
        }

        const monitorTopic = new ROSLIB.Topic({
            ros: rosRef.current,
            name: topicMonitorName,
            messageType: topicMonitorType,
        });

        topicMonitorSubRef.current = monitorTopic;
        setTopicMonitorInfo(`監視中: ${topicMonitorName}`);

        monitorTopic.subscribe((msg) => {
            setTopicMonitorMessages((prev) => [
                {
                    id: `${Date.now()}-${Math.random().toString(16).slice(2)}`,
                    time: new Date().toLocaleTimeString("ja-JP"),
                    text: JSON.stringify(msg, null, 2),
                },
                ...prev,
            ].slice(0, MONITOR_LIMIT));
        });

        return () => {
            try {
                monitorTopic.unsubscribe();
            } catch (_error) {
            }
            if (topicMonitorSubRef.current === monitorTopic) {
                topicMonitorSubRef.current = null;
            }
        };
    }, [rosConnected, topicMonitorRunning, topicMonitorName, topicMonitorType]);

    const targetDifferential = buildDifferentialPreview(parseNumber(targetInputs.r), parseNumber(targetInputs.z));
    const currentDifferential = buildDifferentialPreview(jointFeedback.r, jointFeedback.z);
    const currentHandWorldDeg = jointFeedback.thetaDeg + jointFeedback.wristDeg;
    const hasFault = Boolean(faultText.trim());
    const isEstop = systemState === "ESTOP";

    const updateTopicConfig = (fieldKey, property, value) => {
        setTopicConfig((prev) => ({
            ...prev,
            [fieldKey]: {
                ...prev[fieldKey],
                [property]: value,
            },
        }));
    };

    const renderOverviewPage = () => (
        <section className="serial-packet-section planner-panel">
            <h2 className="serial-packet-title">概要</h2>
            <p className="serial-packet-hint">
                起動状態、エンコーダ初期化、差動機構の換算値、手先姿勢保持の成立状況をまとめて確認します。
            </p>

            <div className="planner-status-grid soki-summary-grid">
                <div className="planner-status-card">
                    <div className="planner-status-card-top">
                        <span className="status-label">ROS</span>
                        <span className={`status-pill ${rosConnected ? "status-ok" : "status-pending"}`}>{status}</span>
                    </div>
                    <strong>{rosEndpoint.host}:{rosEndpoint.port}</strong>
                </div>
                <div className="planner-status-card">
                    <div className="planner-status-card-top">
                        <span className="status-label">状態</span>
                        <span className={`status-pill ${getStatusClassName(rosConnected, hasFault, isEstop)}`}>{systemState}</span>
                    </div>
                    <strong>{stateReason}</strong>
                </div>
                <div className="planner-status-card">
                    <div className="planner-status-card-top">
                        <span className="status-label">Encoder</span>
                        <span className={`status-pill ${encoderInitialized ? "status-ok" : "status-pending"}`}>{encoderInitialized ? "完了" : "未完了"}</span>
                    </div>
                    <strong>r/theta/z/wrist</strong>
                </div>
                <div className="planner-status-card">
                    <div className="planner-status-card-top">
                        <span className="status-label">Motion</span>
                        <span className={`status-pill ${isMoving ? "status-pending" : "status-ok"}`}>{isMoving ? "Moving" : "Idle"}</span>
                    </div>
                    <strong>{targetReached ? "目標近傍" : "追従中"}</strong>
                </div>
            </div>

            <div className="soki-axis-grid">
                <AxisStatusCard
                    label="r"
                    unit="m"
                    currentValue={jointFeedback.r}
                    targetValue={parseNumber(targetInputs.r)}
                    errorValue={targetErrors.r}
                    rawValue={encoderRaw.r}
                    ready={encoderReady.r}
                    digits={3}
                />
                <AxisStatusCard
                    label="theta"
                    unit="deg"
                    currentValue={jointFeedback.thetaDeg}
                    targetValue={parseNumber(targetInputs.theta)}
                    errorValue={targetErrors.thetaDeg}
                    rawValue={encoderRaw.theta}
                    ready={encoderReady.theta}
                    digits={1}
                />
                <AxisStatusCard
                    label="z"
                    unit="m"
                    currentValue={jointFeedback.z}
                    targetValue={parseNumber(targetInputs.z)}
                    errorValue={targetErrors.z}
                    rawValue={encoderRaw.z}
                    ready={encoderReady.z}
                    digits={3}
                />
                <AxisStatusCard
                    label="ハンド姿勢"
                    unit="deg"
                    currentValue={currentHandWorldDeg}
                    targetValue={targetPayload.display.handWorldDeg}
                    errorValue={targetPayload.display.handWorldDeg - currentHandWorldDeg}
                    rawValue={encoderRaw.wrist}
                    ready={encoderReady.wrist}
                    digits={1}
                />
            </div>

            <div className="soki-two-column-grid">
                <div className="serial-packet-section soki-inner-card">
                    <h3 className="serial-packet-title">差動機構プレビュー</h3>
                    <div className="soki-kv-grid">
                        <span>目標 diff A</span>
                        <strong>{formatFixed(targetDifferential.diffA, 3)}</strong>
                        <span>目標 diff B</span>
                        <strong>{formatFixed(targetDifferential.diffB, 3)}</strong>
                        <span>現在 diff A</span>
                        <strong>{formatFixed(currentDifferential.diffA, 3)}</strong>
                        <span>現在 diff B</span>
                        <strong>{formatFixed(currentDifferential.diffB, 3)}</strong>
                    </div>
                    <p className="connection-hint">
                        GUI上では diffA = z + r, diffB = z - r として換算表示しています。実機側で符号系が異なる場合はトピック側で吸収してください。
                    </p>
                </div>

                <div className="serial-packet-section soki-inner-card">
                    <h3 className="serial-packet-title">手先姿勢保持</h3>
                    <div className="soki-kv-grid">
                        <span>保持モード</span>
                        <strong>{holdHandOrientation ? "ON" : "OFF"}</strong>
                        <span>目標世界角</span>
                        <strong>{formatFixed(targetPayload.display.handWorldDeg, 1)} deg</strong>
                        <span>指令ローカル角</span>
                        <strong>{formatFixed(targetPayload.display.wristLocalDeg, 1)} deg</strong>
                        <span>現在世界角</span>
                        <strong>{formatFixed(currentHandWorldDeg, 1)} deg</strong>
                        <span>FB更新時刻</span>
                        <strong>{feedbackUpdatedAt || "未受信"}</strong>
                    </div>
                </div>
            </div>

            <div className="serial-packet-section soki-inner-card">
                <h3 className="serial-packet-title">イベントログ</h3>
                <div className="soki-log-list">
                    {logs.length === 0 ? <div className="planner-debug-empty">まだログはありません</div> : logs.slice(0, 12).map((entry) => (
                        <div className="soki-log-item" key={entry.id}>
                            <span className="soki-log-meta">[{entry.time}] {entry.kind}</span>
                            <span>{entry.message}</span>
                        </div>
                    ))}
                </div>
            </div>
        </section>
    );

    const renderControlPage = () => (
        <section className="serial-packet-section planner-panel">
            <h2 className="serial-packet-title">座標制御</h2>
            <p className="serial-packet-hint">
                r/theta/z と手先姿勢をまとめて指定し、自動移動指令を送ります。ハンド姿勢保持ON時は theta に追従して手先の向きを保ちます。
            </p>

            <div className="control-toggle-row">
                <button
                    type="button"
                    className={`toggle-button ${holdHandOrientation ? "toggle-on" : "toggle-off"}`}
                    onClick={() => setHoldHandOrientation((prev) => !prev)}
                >
                    手先姿勢保持: {holdHandOrientation ? "ON" : "OFF"}
                </button>
                <button
                    type="button"
                    className={`toggle-button ${operationArmed ? "toggle-on" : "toggle-off"}`}
                    onClick={() => setOperationArmed((prev) => !prev)}
                >
                    操作ロック: {operationArmed ? "解除" : "有効"}
                </button>
            </div>

            <div className="soki-target-grid">
                {AXIS_DEFINITIONS.map((axis) => {
                    const inputKey = axis.key === "thetaDeg"
                        ? "theta"
                        : axis.key === "wristDeg"
                            ? "hand"
                            : axis.key;

                    return (
                        <div className="serial-item" key={axis.key}>
                            <span className="serial-item-name">{axis.label}</span>
                            <span className="serial-item-desc">単位: {axis.unit}</span>
                            <input
                                className="connection-input"
                                type="number"
                                step="any"
                                value={targetInputs[inputKey]}
                                onChange={(event) => updateTargetInput(inputKey, event.target.value)}
                            />
                            <div className="soki-stepper-row">
                                <button type="button" className="connection-button btn-neutral" onClick={() => nudgeTarget(inputKey, -1)}>-</button>
                                <input
                                    className="connection-input"
                                    type="number"
                                    step="any"
                                    value={targetSteps[axis.stepKey]}
                                    onChange={(event) => updateTargetStep(axis.stepKey, event.target.value)}
                                />
                                <button type="button" className="connection-button btn-neutral" onClick={() => nudgeTarget(inputKey, 1)}>+</button>
                            </div>
                        </div>
                    );
                })}
            </div>

            <div className="serial-packet-actions soki-action-grid">
                <button type="button" className="connection-button btn-connect" onClick={copyCurrentToTarget}>現在値を目標へ</button>
                <button type="button" className="connection-button btn-save" onClick={saveTargetPreset}>目標を保存</button>
                <button type="button" className="connection-button btn-send" onClick={() => sendTargetCommand(false)}>目標のみ送信</button>
                <button type="button" className="connection-button btn-send" onClick={() => sendTargetCommand(true)}>自動移動開始</button>
            </div>

            <div className="serial-packet-section soki-inner-card">
                <h3 className="serial-packet-title">自動移動判定</h3>
                <div className="soki-kv-grid soki-tolerance-grid">
                    <span>判定状態</span>
                    <strong>{targetReached ? "目標近傍" : "未到達"}</strong>
                    <span>r 許容差</span>
                    <input className="connection-input" type="number" step="any" value={targetTolerances.r} onChange={(event) => updateTolerance("r", event.target.value)} />
                    <span>theta 許容差</span>
                    <input className="connection-input" type="number" step="any" value={targetTolerances.theta} onChange={(event) => updateTolerance("theta", event.target.value)} />
                    <span>z 許容差</span>
                    <input className="connection-input" type="number" step="any" value={targetTolerances.z} onChange={(event) => updateTolerance("z", event.target.value)} />
                    <span>wrist 許容差</span>
                    <input className="connection-input" type="number" step="any" value={targetTolerances.wrist} onChange={(event) => updateTolerance("wrist", event.target.value)} />
                </div>
                <p className="connection-hint">AUTO_MOVE 中に全軸が許容差内へ入ると GUI 側で HOLD へ遷移します。</p>
            </div>

            <div className="serial-packet-section soki-inner-card">
                <div className="soki-preset-header">
                    <h3 className="serial-packet-title">保存プリセット</h3>
                    <input
                        className="connection-input"
                        type="text"
                        value={presetNameInput}
                        onChange={(event) => setPresetNameInput(event.target.value)}
                        placeholder="プリセット名"
                    />
                </div>
                <div className="soki-preset-list">
                    {savedTargets.length === 0 ? <div className="planner-debug-empty">保存済みプリセットはありません</div> : savedTargets.map((preset) => (
                        <div className="soki-preset-item" key={preset.id}>
                            <div>
                                <strong>{preset.label}</strong>
                                <p>{preset.summary}</p>
                                <span>{preset.createdAt}</span>
                            </div>
                            <div className="soki-inline-actions">
                                <button type="button" className="connection-button btn-restore" onClick={() => applyPreset(preset, false)}>復元</button>
                                <button type="button" className="connection-button btn-send" onClick={() => applyPreset(preset, true)}>復元して送信</button>
                                <button type="button" className="connection-button btn-neutral" onClick={() => deletePreset(preset.id)}>削除</button>
                            </div>
                        </div>
                    ))}
                </div>
            </div>
        </section>
    );

    const renderAutomationPage = () => (
        <section className="serial-packet-section planner-panel">
            <h2 className="serial-packet-title">自動化</h2>
            <p className="serial-packet-hint">
                起動時のエンコーダ初期化、状態管理、指定座標への自動移動をここで運用します。デバッグ時は任意状態へ手動遷移できます。
            </p>

            <div className="control-toggle-row">
                <button
                    type="button"
                    className={`toggle-button ${autoInitOnConnect ? "toggle-on" : "toggle-off"}`}
                    onClick={() => setAutoInitOnConnect((prev) => !prev)}
                >
                    接続時エンコーダ初期化: {autoInitOnConnect ? "ON" : "OFF"}
                </button>
                <button type="button" className="connection-button btn-connect" onClick={() => requestEncoderInitialization("手動で初期化要求")}>初期化を再実行</button>
            </div>

            <div className="planner-state-buttons soki-state-buttons">
                {STATE_DEFINITIONS.map((state) => (
                    <button
                        key={state.key}
                        type="button"
                        className={`planner-choice-button planner-state-choice ${systemState === state.key ? "planner-choice-selected" : ""}`}
                        onClick={() => transitionState(state.key, `GUIから ${state.label} を選択`, true)}
                    >
                        {state.label}
                    </button>
                ))}
            </div>

            <div className="planner-status-grid soki-summary-grid">
                <div className="planner-status-card">
                    <div className="planner-status-card-top">
                        <span className="status-label">状態</span>
                        <span className={`status-pill ${getStatusClassName(rosConnected, hasFault, isEstop)}`}>{systemState}</span>
                    </div>
                    <strong>{stateReason}</strong>
                </div>
                <div className="planner-status-card">
                    <div className="planner-status-card-top">
                        <span className="status-label">Encoder初期化</span>
                        <span className={`status-pill ${encoderInitialized ? "status-ok" : "status-pending"}`}>{encoderInitialized ? "Ready" : "Pending"}</span>
                    </div>
                    <strong>{encoderReady.r ? "r " : ""}{encoderReady.theta ? "theta " : ""}{encoderReady.z ? "z " : ""}{encoderReady.wrist ? "wrist" : ""}</strong>
                </div>
                <div className="planner-status-card">
                    <div className="planner-status-card-top">
                        <span className="status-label">目標到達</span>
                        <span className={`status-pill ${targetReached ? "status-ok" : "status-pending"}`}>{targetReached ? "Yes" : "No"}</span>
                    </div>
                    <strong>{isMoving ? "移動中" : "停止中"}</strong>
                </div>
                <div className="planner-status-card">
                    <div className="planner-status-card-top">
                        <span className="status-label">Fault</span>
                        <span className={`status-pill ${hasFault ? "status-bad" : "status-ok"}`}>{hasFault ? "検出" : "なし"}</span>
                    </div>
                    <strong>{faultText || "No fault"}</strong>
                </div>
            </div>

            <div className="serial-packet-section soki-inner-card">
                <h3 className="serial-packet-title">運用チェック</h3>
                <div className="soki-check-grid">
                    <div className="status-row"><span className="status-label">ROS接続</span><span className={`status-pill ${rosConnected ? "status-ok" : "status-bad"}`}>{rosConnected ? "OK" : "NG"}</span></div>
                    <div className="status-row"><span className="status-label">操作ロック解除</span><span className={`status-pill ${operationArmed ? "status-ok" : "status-pending"}`}>{operationArmed ? "Ready" : "Lock"}</span></div>
                    <div className="status-row"><span className="status-label">Encoder初期化</span><span className={`status-pill ${encoderInitialized ? "status-ok" : "status-pending"}`}>{encoderInitialized ? "OK" : "Pending"}</span></div>
                    <div className="status-row"><span className="status-label">Fault無し</span><span className={`status-pill ${hasFault ? "status-bad" : "status-ok"}`}>{hasFault ? "Fault" : "Clear"}</span></div>
                </div>
            </div>
        </section>
    );

    const renderActuatorPage = () => (
        <section className="serial-packet-section planner-panel">
            <h2 className="serial-packet-title">アクチュエータ</h2>
            <p className="serial-packet-hint">
                本番系は座標指令を優先し、ここはデバッグ用の直接出力です。theta 1軸、r/z差動 2軸、手先姿勢 1軸、ハンド 1軸を想定しています。
            </p>

            <div className="serial-packet-grid">
                {[
                    ["theta", "theta motor"],
                    ["diffA", "r/z differential A"],
                    ["diffB", "r/z differential B"],
                    ["wrist", "hand orientation"],
                    ["gripper", "hand actuator"],
                ].map(([key, label]) => (
                    <label className="serial-item" key={key}>
                        <span className="serial-item-name">{label}</span>
                        <span className="serial-item-desc">デバッグ指令値</span>
                        <input
                            className="connection-input"
                            type="number"
                            step="any"
                            value={motorCommandInputs[key]}
                            onChange={(event) => setMotorCommandInputs((prev) => ({ ...prev, [key]: event.target.value }))}
                        />
                    </label>
                ))}
            </div>

            <div className="serial-packet-actions soki-action-grid">
                <button type="button" className="connection-button btn-send" onClick={sendMotorDebugCommand}>デバッグ出力送信</button>
                <button type="button" className="connection-button btn-send" onClick={sendGripperCommand}>ハンドのみ送信</button>
                <button type="button" className="serial-clear-button" onClick={clearMotorCommands}>ゼロクリア</button>
            </div>

            <div className="soki-two-column-grid">
                <div className="serial-packet-section soki-inner-card">
                    <h3 className="serial-packet-title">モータFB</h3>
                    <div className="soki-kv-grid">
                        <span>theta</span>
                        <strong>{formatFixed(motorFeedback.theta, 3)}</strong>
                        <span>diff A</span>
                        <strong>{formatFixed(motorFeedback.diffA, 3)}</strong>
                        <span>diff B</span>
                        <strong>{formatFixed(motorFeedback.diffB, 3)}</strong>
                        <span>wrist</span>
                        <strong>{formatFixed(motorFeedback.wrist, 3)}</strong>
                        <span>gripper</span>
                        <strong>{formatFixed(motorFeedback.gripper, 3)}</strong>
                        <span>更新時刻</span>
                        <strong>{motorUpdatedAt || "未受信"}</strong>
                    </div>
                </div>

                <div className="serial-packet-section soki-inner-card">
                    <h3 className="serial-packet-title">座標系との対応</h3>
                    <div className="soki-kv-grid">
                        <span>theta目標</span>
                        <strong>{targetInputs.theta} deg</strong>
                        <span>r目標</span>
                        <strong>{targetInputs.r} m</strong>
                        <span>z目標</span>
                        <strong>{targetInputs.z} m</strong>
                        <span>姿勢指令</span>
                        <strong>{formatFixed(targetPayload.display.wristLocalDeg, 1)} deg</strong>
                        <span>ハンド</span>
                        <strong>{targetInputs.gripper}</strong>
                    </div>
                </div>
            </div>
        </section>
    );

    const renderEscPage = () => (
        <section className="serial-packet-section planner-panel">
            <h2 className="serial-packet-title">ESC設定</h2>
            <p className="serial-packet-hint">
                B-G431B-ESC1(SimpleFOC)の実行コマンドとゲイン/リミットを送信します。送信は操作ロック解除(ARMED)が必要です。
            </p>

            <div className="serial-packet-section soki-inner-card">
                <h3 className="serial-packet-title">コマンド</h3>
                <div className="serial-packet-grid">
                    <label className="serial-item">
                        <span className="serial-item-name">enable</span>
                        <span className="serial-item-desc">0=stop, 1=run</span>
                        <input
                            className="connection-input"
                            type="number"
                            step="1"
                            value={escCommandInputs.enable}
                            onChange={(event) => setEscCommandInputs((prev) => ({ ...prev, enable: event.target.value }))}
                        />
                    </label>
                    <label className="serial-item">
                        <span className="serial-item-name">mode</span>
                        <span className="serial-item-desc">0=velocity, 1=angle</span>
                        <input
                            className="connection-input"
                            type="number"
                            step="1"
                            value={escCommandInputs.mode}
                            onChange={(event) => setEscCommandInputs((prev) => ({ ...prev, mode: event.target.value }))}
                        />
                    </label>
                    <label className="serial-item">
                        <span className="serial-item-name">target</span>
                        <span className="serial-item-desc">mode=0: rad/s, mode=1: deg</span>
                        <input
                            className="connection-input"
                            type="number"
                            step="any"
                            value={escCommandInputs.target}
                            onChange={(event) => setEscCommandInputs((prev) => ({ ...prev, target: event.target.value }))}
                        />
                    </label>
                    <label className="serial-item">
                        <span className="serial-item-name">voltage_limit</span>
                        <span className="serial-item-desc">V</span>
                        <input
                            className="connection-input"
                            type="number"
                            step="any"
                            value={escCommandInputs.voltageLimit}
                            onChange={(event) => setEscCommandInputs((prev) => ({ ...prev, voltageLimit: event.target.value }))}
                        />
                    </label>
                </div>

                <div className="serial-packet-actions soki-action-grid">
                    <button type="button" className="connection-button btn-send" onClick={sendEscCommand}>コマンド送信</button>
                </div>
            </div>

            <div className="serial-packet-section soki-inner-card">
                <h3 className="serial-packet-title">ゲイン/リミット</h3>
                <div className="serial-packet-grid">
                    <label className="serial-item">
                        <span className="serial-item-name">velocity_limit</span>
                        <span className="serial-item-desc">rad/s</span>
                        <input
                            className="connection-input"
                            type="number"
                            step="any"
                            value={escParamInputs.velocityLimit}
                            onChange={(event) => setEscParamInputs((prev) => ({ ...prev, velocityLimit: event.target.value }))}
                        />
                    </label>
                    <label className="serial-item">
                        <span className="serial-item-name">current_limit</span>
                        <span className="serial-item-desc">A</span>
                        <input
                            className="connection-input"
                            type="number"
                            step="any"
                            value={escParamInputs.currentLimit}
                            onChange={(event) => setEscParamInputs((prev) => ({ ...prev, currentLimit: event.target.value }))}
                        />
                    </label>
                    <label className="serial-item">
                        <span className="serial-item-name">PID_velocity.P</span>
                        <span className="serial-item-desc">P gain</span>
                        <input
                            className="connection-input"
                            type="number"
                            step="any"
                            value={escParamInputs.velP}
                            onChange={(event) => setEscParamInputs((prev) => ({ ...prev, velP: event.target.value }))}
                        />
                    </label>
                    <label className="serial-item">
                        <span className="serial-item-name">PID_velocity.I</span>
                        <span className="serial-item-desc">I gain</span>
                        <input
                            className="connection-input"
                            type="number"
                            step="any"
                            value={escParamInputs.velI}
                            onChange={(event) => setEscParamInputs((prev) => ({ ...prev, velI: event.target.value }))}
                        />
                    </label>
                    <label className="serial-item">
                        <span className="serial-item-name">PID_velocity.D</span>
                        <span className="serial-item-desc">D gain</span>
                        <input
                            className="connection-input"
                            type="number"
                            step="any"
                            value={escParamInputs.velD}
                            onChange={(event) => setEscParamInputs((prev) => ({ ...prev, velD: event.target.value }))}
                        />
                    </label>
                    <label className="serial-item">
                        <span className="serial-item-name">PID_velocity.output_ramp</span>
                        <span className="serial-item-desc">SimpleFOC</span>
                        <input
                            className="connection-input"
                            type="number"
                            step="any"
                            value={escParamInputs.velRamp}
                            onChange={(event) => setEscParamInputs((prev) => ({ ...prev, velRamp: event.target.value }))}
                        />
                    </label>
                    <label className="serial-item">
                        <span className="serial-item-name">LPF_velocity.Tf</span>
                        <span className="serial-item-desc">s</span>
                        <input
                            className="connection-input"
                            type="number"
                            step="any"
                            value={escParamInputs.velLpfTf}
                            onChange={(event) => setEscParamInputs((prev) => ({ ...prev, velLpfTf: event.target.value }))}
                        />
                    </label>
                    <label className="serial-item">
                        <span className="serial-item-name">P_angle.P</span>
                        <span className="serial-item-desc">angle P gain</span>
                        <input
                            className="connection-input"
                            type="number"
                            step="any"
                            value={escParamInputs.angleP}
                            onChange={(event) => setEscParamInputs((prev) => ({ ...prev, angleP: event.target.value }))}
                        />
                    </label>
                </div>

                <div className="serial-packet-actions soki-action-grid">
                    <button type="button" className="connection-button btn-send" onClick={sendEscParams}>パラメータ送信</button>
                </div>
            </div>

            <div className="serial-packet-section soki-inner-card">
                <h3 className="serial-packet-title">フィードバック (serial_rx)</h3>
                <div className="soki-kv-grid">
                    <span>mode</span>
                    <strong>{escFeedback.mode === 1 ? "angle" : "velocity"}</strong>
                    <span>angle</span>
                    <strong>{formatFixed(escFeedback.angleDeg, 1)} deg</strong>
                    <span>velocity</span>
                    <strong>{formatFixed(escFeedback.velocityRadS, 1)} rad/s</strong>
                    <span>target</span>
                    <strong>{formatFixed(escFeedback.target, 1)}</strong>
                    <span>vlim</span>
                    <strong>{formatFixed(escFeedback.voltageLimitV, 1)} V</strong>
                    <span>rpm</span>
                    <strong>{escFeedback.rpm}</strong>
                    <span>params</span>
                    <strong>{escInfo}</strong>
                </div>

                {escFeedback.velocityPid && (
                    <p className="connection-hint">
                        echo: vel_lim={escFeedback.velocityLimitRadS?.toFixed(1)} rad/s, cur_lim={escFeedback.currentLimitA?.toFixed(1)} A, velPID(P/I/D)={escFeedback.velocityPid.p.toFixed(3)}/{escFeedback.velocityPid.i.toFixed(3)}/{escFeedback.velocityPid.d.toFixed(3)}, ramp={escFeedback.velocityOutputRamp}, Tf={escFeedback.velocityLpfTfS?.toFixed(3)}s, angP={escFeedback.anglePGain?.toFixed(2)}
                    </p>
                )}
            </div>
        </section>
    );

    const renderDebugPage = () => (
        <section className="serial-packet-section planner-panel">
            <h2 className="serial-packet-title">デバッグ</h2>
            <p className="serial-packet-hint">
                任意トピック監視と生メッセージ確認に使います。ROS側のメッセージ定義変更時の切り分けをここで行えます。
            </p>

            <div className="serial-packet-controls soki-monitor-controls">
                <label className="serial-packet-label">
                    <span>監視トピック</span>
                    <input className="connection-input" type="text" value={topicMonitorName} onChange={(event) => setTopicMonitorName(event.target.value)} />
                </label>
                <label className="serial-packet-label soki-monitor-type">
                    <span>型</span>
                    <input className="connection-input" type="text" value={topicMonitorType} onChange={(event) => setTopicMonitorType(event.target.value)} />
                </label>
                <div className="serial-packet-actions">
                    <button type="button" className="connection-button btn-connect" onClick={() => setTopicMonitorRunning((prev) => !prev)}>
                        {topicMonitorRunning ? "監視停止" : "監視開始"}
                    </button>
                </div>
            </div>

            <div className="status-row">
                <span className="status-label">Topic monitor</span>
                <span className={`status-pill ${topicMonitorRunning ? "status-ok" : "status-pending"}`}>{topicMonitorInfo}</span>
            </div>

            <div className="soki-two-column-grid">
                <div className="serial-packet-section soki-inner-card">
                    <h3 className="serial-packet-title">Topic Echo</h3>
                    <div className="soki-raw-list">
                        {topicMonitorMessages.length === 0 ? <div className="planner-debug-empty">まだ受信していません</div> : topicMonitorMessages.map((entry) => (
                            <div className="soki-raw-item" key={entry.id}>
                                <div className="soki-log-meta">[{entry.time}]</div>
                                <pre>{entry.text}</pre>
                            </div>
                        ))}
                    </div>
                </div>

                <div className="serial-packet-section soki-inner-card">
                    <h3 className="serial-packet-title">受信スナップショット</h3>
                    <div className="soki-raw-list">
                        {Object.entries(debugSnapshots).map(([key, value]) => (
                            <div className="soki-raw-item" key={key}>
                                <strong>{key}</strong>
                                <pre>{value || "未受信"}</pre>
                            </div>
                        ))}
                    </div>
                </div>
            </div>
        </section>
    );

    const renderSettingsPage = () => (
        <section className="serial-packet-section planner-panel">
            <h2 className="serial-packet-title">設定</h2>
            <p className="serial-packet-hint">
                rosbridge 接続先とトピック名を管理します。実機側のインタフェースが固まり次第、ここを書き換えればGUIを保ったまま追従できます。
            </p>

            <div className="connection-status-bar">
                <div className="connection-status-inputs">
                    <input
                        className="connection-input"
                        type="text"
                        value={rosHostInput}
                        onChange={(event) => setRosHostInput(event.target.value)}
                        placeholder="ROS host"
                    />
                    <input
                        className="connection-input connection-port"
                        type="text"
                        value={rosPortInput}
                        onChange={(event) => setRosPortInput(event.target.value)}
                        placeholder="9090"
                    />
                </div>
                <div className="connection-status-meta">
                    <span className="connection-compact-url">{settingsInfo}</span>
                    <button type="button" className="connection-button btn-connect" onClick={applyConnectionSettings}>接続更新</button>
                </div>
            </div>

            <div className="soki-topic-config-list">
                {TOPIC_FIELD_DEFINITIONS.map((field) => (
                    <div className="soki-topic-config-row" key={field.key}>
                        <span className="soki-topic-label">{field.label}</span>
                        <input
                            className="connection-input"
                            type="text"
                            value={topicConfig[field.key].name}
                            onChange={(event) => updateTopicConfig(field.key, "name", event.target.value)}
                        />
                        <input
                            className="connection-input"
                            type="text"
                            value={topicConfig[field.key].type}
                            onChange={(event) => updateTopicConfig(field.key, "type", event.target.value)}
                        />
                    </div>
                ))}
            </div>
        </section>
    );

    return (
        <div className="console-page">
            <div className="console-bg-shape console-bg-shape-a" />
            <div className="console-bg-shape console-bg-shape-b" />

            <main className="console-card">
                <header className="console-header">
                    <div className="console-logo soki-logo-mark">SO</div>
                    <div>
                        <h1>SOKI Console</h1>
                        <p>r / theta / z + hand manipulator console</p>
                        <div className="console-version-text">UI VERSION {GUI_VERSION}</div>
                    </div>
                    <div className="header-tools soki-header-tools">
                        <button
                            type="button"
                            className={`toggle-button ${multiTabMode ? "toggle-on" : "toggle-off"}`}
                            onClick={() => setMultiTabMode((prev) => !prev)}
                        >
                            Multi View: {multiTabMode ? "ON" : "OFF"}
                        </button>
                        <button type="button" className="connection-button btn-neutral" onClick={sendEmergencyStop}>非常停止</button>
                    </div>
                </header>

                <div className="status-row">
                    <span className="status-label">接続 / 状態 / 操作ロック</span>
                    <span className={`status-pill ${getStatusClassName(rosConnected, hasFault, isEstop)}`}>
                        {status} / {systemState} / {operationArmed ? "ARMED" : "LOCKED"}
                    </span>
                </div>

                <div className="page-switch-row">
                    {PAGE_DEFINITIONS.map((page) => {
                        const active = activePages.includes(page.key);
                        return (
                            <button
                                key={page.key}
                                type="button"
                                className={`toggle-button ${active ? "toggle-on" : "toggle-off"}`}
                                onClick={() => handlePageToggle(page.key)}
                            >
                                {page.label}
                            </button>
                        );
                    })}
                </div>

                <div className="active-pages-grid">
                    {activePages.includes("overview") && renderOverviewPage()}
                    {activePages.includes("control") && renderControlPage()}
                    {activePages.includes("automation") && renderAutomationPage()}
                    {activePages.includes("actuator") && renderActuatorPage()}
                    {activePages.includes("esc") && renderEscPage()}
                    {activePages.includes("debug") && renderDebugPage()}
                    {activePages.includes("settings") && renderSettingsPage()}
                </div>

                <div className="status-row">
                    <span className="status-label">操作結果</span>
                    <span className="status-pill status-pending">{motionInfo}</span>
                </div>
                <div className="status-row">
                    <span className="status-label">アクチュエータ</span>
                    <span className="status-pill status-pending">{actuatorInfo}</span>
                </div>
            </main>
        </div>
    );
}

export default App;
