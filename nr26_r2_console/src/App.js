import React, { useEffect, useRef, useState } from "react";
import * as ROSLIB from "roslib";
import "./App.css";
import { getLocalizedText, translateRuntimeText as translateRuntimeByLanguage } from "./i18n";
import { LANGUAGE_OPTIONS } from "./i18n";
import appPackage from "../package.json";

const GUI_VERSION = process.env.REACT_APP_UI_VERSION || appPackage.version || "0.0.0";

const PACKET_INDEX_LABELS = [
  "DEBUG",
  "MD1", "MD2", "MD3", "MD4", "MD5", "MD6", "MD7", "MD8",
  "SERVO1", "SERVO2", "SERVO3", "SERVO4", "SERVO5", "SERVO6", "SERVO7", "SERVO8",
  "TR1", "TR2", "TR3", "TR4", "TR5", "TR6", "TR7",
];

const DEFAULT_PACKET_COUNT = 24;
const SERIAL_BRIDGE_MIN_ELEMENTS = 24;

const describeActuatorJa = (label) => {
  if (label === "DEBUG") return "デバッグ (0/1)";
  if (label.startsWith("MD")) return "モータ出力 (-255〜255)";
  if (label.startsWith("SERVO")) return "サーボ角度 (0〜270)";
  if (label.startsWith("TR")) return "デジタル出力 (0/1)";
  return "予備";
};

const describeActuatorEn = (label) => {
  if (label === "DEBUG") return "Debug (0/1)";
  if (label.startsWith("MD")) return "Motor output (-255 to 255)";
  if (label.startsWith("SERVO")) return "Servo angle (0 to 270)";
  if (label.startsWith("TR")) return "Digital output (0/1)";
  return "予備";
};

const normalizeYawRad = (yawRad) => {
  if (!Number.isFinite(yawRad)) {
    return 0;
  }
  const wrapped = ((yawRad + Math.PI) % (Math.PI * 2) + Math.PI * 2) % (Math.PI * 2);
  return wrapped - Math.PI;
};

const normalizeAngleDeg = (deg) => {
  if (!Number.isFinite(deg)) {
    return 0;
  }
  const wrapped = ((deg + 180) % 360 + 360) % 360;
  return wrapped - 180;
};

const chooseGridStep = (span) => {
  const candidates = [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10];
  const target = Math.max(span / 6, 0.05);
  for (const step of candidates) {
    if (step >= target) {
      return step;
    }
  }
  return 20;
};

const PLANNER_STATE_LABELS = {
  0: { ja: "待機", en: "Waiting" },
  1: { ja: "MFF進入", en: "Enter MFF" },
  2: { ja: "MFF離脱", en: "Leave MFF" },
  3: { ja: "スタッフ組み立て", en: "Staff Assembly" },
  4: { ja: "ラック移動", en: "Rack Move" },
  5: { ja: "スタッフハンドトリガー", en: "Staff Hand Trigger" },
};

const BUILTIN_PLANNER_STATE_CODES = [0, 4, 5, 3, 1, 2];

const DRIVE_MODE_LABELS = {
  0: { ja: "手動", en: "Manual" },
  1: { ja: "自動", en: "Auto" },
  2: { ja: "ArUco", en: "ArUco" },
};

const PLANNER_COLOR_LABELS = {
  [-1]: { ja: "未設定", en: "Unknown" },
  0: { ja: "青コート", en: "Blue" },
  1: { ja: "赤コート", en: "Red" },
};

const PLANNER_MODE_LABELS = {
  0: { ja: "手動遷移", en: "Manual" },
  1: { ja: "自動遷移", en: "Auto" },
};

const getPlannerStateLabel = (code, language, customStateLabelMap = null) => {
  const customEntry = customStateLabelMap ? customStateLabelMap[code] : null;
  if (customEntry) {
    return language === "ja" ? customEntry.ja : customEntry.en;
  }
  const entry = PLANNER_STATE_LABELS[code];
  return entry ? (language === "ja" ? entry.ja : entry.en) : `${language === "ja" ? "状態" : "State"} ${code}`;
};

const getPlannerColorLabel = (code, language) => {
  const entry = PLANNER_COLOR_LABELS[code];
  return entry ? (language === "ja" ? entry.ja : entry.en) : `${language === "ja" ? "色" : "Color"} ${code}`;
};

const getPlannerModeLabel = (code, language) => {
  const entry = PLANNER_MODE_LABELS[code];
  return entry ? (language === "ja" ? entry.ja : entry.en) : `${language === "ja" ? "モード" : "Mode"} ${code}`;
};

const getDriveModeLabel = (code, language) => {
  const entry = DRIVE_MODE_LABELS[code];
  return entry ? (language === "ja" ? entry.ja : entry.en) : `${language === "ja" ? "モード" : "Mode"} ${code}`;
};

const createPlannerStatePoseConfig = (stateCodes = BUILTIN_PLANNER_STATE_CODES) =>
  Object.fromEntries(
    stateCodes.map((stateCode) => [
      stateCode,
      {
        enabled: false,
        x: "0.0",
        y: "0.0",
        yaw: "0.0",
      },
    ])
  );

const createPlannerStateModeConfig = (stateCodes = BUILTIN_PLANNER_STATE_CODES) =>
  Object.fromEntries(
    stateCodes.map((stateCode) => [
      stateCode,
      {
        enabled: false,
        modeCode: 1,
      },
    ])
  );

const createPlannerStateOdomResetConfig = (stateCodes = BUILTIN_PLANNER_STATE_CODES) =>
  Object.fromEntries(
    stateCodes.map((stateCode) => [
      stateCode,
      {
        enabled: false,
      },
    ])
  );

const MFF_LAYOUT_RED = [
  [1, 2, 3],
  [4, 5, 6],
  [7, 8, 9],
  [10, 11, 12],
];

const MFF_LAYOUT_BLUE = [
  [3, 2, 1],
  [6, 5, 4],
  [9, 8, 7],
  [12, 11, 10],
];

const getMffLayout = (colorCode) => {
  if (colorCode === 1) {
    return MFF_LAYOUT_RED;
  }
  if (colorCode === 0) {
    return MFF_LAYOUT_BLUE;
  }
  return MFF_LAYOUT_RED;
};

// Height groups (mm) based on the shared field diagram.
const MFF_HEIGHT_LEVEL_BY_CELL = {
  1: "mm-200",
  2: "mm-0",
  3: "mm-200",
  4: "mm-0",
  5: "mm-200",
  6: "mm-400",
  7: "mm-200",
  8: "mm-400",
  9: "mm-200",
  10: "mm-0",
  11: "mm-200",
  12: "mm-0",
};

const getMffHeightLevel = (cellNumber) => MFF_HEIGHT_LEVEL_BY_CELL[cellNumber] || "mm-200";

const PLANNER_FIELD_ROTATION_STEP_DEG = 90;

function App() {
  const rosRef = useRef(null);
  const commandRef = useRef(null);
  const joyRef = useRef(null);
  const virtualOdomPubRef = useRef(null);
  const virtualOdomTimerRef = useRef(null);
  const odomRef = useRef(null);
  const driveModeRef = useRef(null);
  const autoDriveCmdRef = useRef(null);
  const arucoCameraOffsetRef = useRef(null);
  const arucoTargetIdRef = useRef(null);
  const odomResetCmdRef = useRef(null);
  const taskStateCommandRef = useRef(null);
  const taskColorCommandRef = useRef(null);
  const taskCellCommandRef = useRef(null);
  const taskTransitionModeRef = useRef(null);
  const taskStateSequenceRef = useRef(null);
  const taskStatePoseRef = useRef(null);
  const taskStateModeRef = useRef(null);
  const taskStateOdomResetRef = useRef(null);
  const taskAutoSendEnabledRef = useRef(null);
  const taskStatusSubRef = useRef(null);
  const taskStatusTextSubRef = useRef(null);
  const rosTopicsServiceRef = useRef(null);
  const rosTopicTypeServiceRef = useRef(null);
  const topicEchoSubRef = useRef(null);
  const cameraSubRef = useRef(null);
  const cubeDebugSubRef = useRef(null);
  const wallAngleSubRef = useRef(null);
  const arucoPoseSubRef = useRef(null);
  const arucoIdSubRef = useRef(null);
  const serialPeriodicTimerRef = useRef(null);
  const serialBridgeLogBoxRef = useRef(null);
  const poseGraphRef = useRef(null);
  const poseGraphPointerActiveRef = useRef(false);
  const waypointGraphRef = useRef(null);
  const traceRecordTimerRef = useRef(null);
  const traceReplayTimerRef = useRef(null);
  const traceImportInputRef = useRef(null);
  const plannerConfigImportInputRef = useRef(null);
  const tracePoseRef = useRef({ x: 0, y: 0, yaw: 0 });
  const traceLanguageRef = useRef("ja");
  const defaultRosHost = window.location.hostname || "localhost";
  const wsScheme = window.location.protocol === "https:" ? "wss" : "ws";
  const commandValueRef = useRef(0);
  const buttonsRef = useRef(Array(14).fill(0));
  const axesRef = useRef(Array(8).fill(0));

  const [status, setStatus] = useState("接続中...");
  const [rosHostInput, setRosHostInput] = useState(defaultRosHost);
  const [rosPortInput, setRosPortInput] = useState("9090");
  const [rosEndpoint, setRosEndpoint] = useState({
    host: defaultRosHost,
    port: "9090",
  });
  const [commandValue, setCommandValue] = useState(0);
  const [buttons, setButtons] = useState(Array(14).fill(0));
  const [axes, setAxes] = useState(Array(8).fill(0));
  const [language, setLanguage] = useState("ja");
  const [operationArmed, setOperationArmed] = useState(false);
  const [frontendForceStopped, setFrontendForceStopped] = useState(false);
  const [controllerEnabled, setControllerEnabled] = useState(false);
  const [controllerFullscreen, setControllerFullscreen] = useState(false);
  const [activePages, setActivePages] = useState(["controller"]);
  const [multiTabMode, setMultiTabMode] = useState(false);
  const pageOrder = [
    "controller",
    "pose",
    "planner",
    "wall-angle",
    "waypoint",
    "teaching",
    "simulator",
    "actuator",
    "topic",
    "camera",
    "cube-detection",
    "serial-bridge",
    "shutdown",
    "settings",
  ];
  const [joyTopicName, setJoyTopicName] = useState("/joy_9");
  const [joyTopicInput, setJoyTopicInput] = useState("/joy_9");
  const [virtualOdomTopicInput, setVirtualOdomTopicInput] = useState("odom_xy_yaw");
  const [virtualOdomTopicName, setVirtualOdomTopicName] = useState("odom_xy_yaw");
  const [serialTargetIdInput, setSerialTargetIdInput] = useState("1");
  const [serialElementCount, setSerialElementCount] = useState(DEFAULT_PACKET_COUNT);
  const [serialValues, setSerialValues] = useState(Array(DEFAULT_PACKET_COUNT).fill(0));
  const [serialPublishInfo, setSerialPublishInfo] = useState("未送信");
  const [serialPeriodicHz, setSerialPeriodicHz] = useState("10");
  const [serialPeriodicEnabled, setSerialPeriodicEnabled] = useState(false);
  const [poseX, setPoseX] = useState(0);
  const [poseY, setPoseY] = useState(0);
  const [poseYaw, setPoseYaw] = useState(0);
  const [driveMode, setDriveMode] = useState("MANUAL");
  const [targetXInput, setTargetXInput] = useState("0.0");
  const [targetYInput, setTargetYInput] = useState("0.0");
  const [targetYawInput, setTargetYawInput] = useState("0.0");
  const [targetXStep, setTargetXStep] = useState("0.1");
  const [targetYStep, setTargetYStep] = useState("0.1");
  const [targetYawStep, setTargetYawStep] = useState("5");
  const [plannerStateCode, setPlannerStateCode] = useState(0);
  const [plannerColorCode, setPlannerColorCode] = useState(-1);
  const [plannerCellCode, setPlannerCellCode] = useState(0);
  const [plannerTransitionModeCode, setPlannerTransitionModeCode] = useState(0);
  const [plannerStatusText, setPlannerStatusText] = useState("state=WAITING color=UNKNOWN cell=0 mode=MANUAL");
  const [plannerCellInput, setPlannerCellInput] = useState("0");
  const [plannerFieldRotationDeg, setPlannerFieldRotationDeg] = useState(180);
  const [plannerAutoSendEnabled, setPlannerAutoSendEnabled] = useState(true);
  const [plannerCustomStates, setPlannerCustomStates] = useState([]);
  const [plannerNewStateCodeInput, setPlannerNewStateCodeInput] = useState("");
  const [plannerNewStateJaInput, setPlannerNewStateJaInput] = useState("");
  const [plannerNewStateEnInput, setPlannerNewStateEnInput] = useState("");
  const [plannerStateSequence, setPlannerStateSequence] = useState(() => [...BUILTIN_PLANNER_STATE_CODES]);
  const [plannerStatePoseConfig, setPlannerStatePoseConfig] = useState(() => createPlannerStatePoseConfig());
  const [plannerStateModeConfig, setPlannerStateModeConfig] = useState(() => createPlannerStateModeConfig());
  const [plannerStateOdomResetConfig, setPlannerStateOdomResetConfig] = useState(() => createPlannerStateOdomResetConfig());
  const [plannerConfigIsDirty, setPlannerConfigIsDirty] = useState(false);
  const [plannerExportsList, setPlannerExportsList] = useState([]);
  const [plannerSelectedExportPath, setPlannerSelectedExportPath] = useState("");
  const [arucoTargetForwardInput, setArucoTargetForwardInput] = useState("0.0");
  const [arucoTargetLateralInput, setArucoTargetLateralInput] = useState("0.0");
  const [arucoTargetYawInput, setArucoTargetYawInput] = useState("0.0");
  const [arucoTargetIdInput, setArucoTargetIdInput] = useState("-1");
  const [arucoCameraOffsetXInput, setArucoCameraOffsetXInput] = useState("-0.1735");
  const [arucoCameraOffsetYInput, setArucoCameraOffsetYInput] = useState("0.0");
  const [arucoCmdInfo, setArucoCmdInfo] = useState("未送信");
  const [arucoDetectedId, setArucoDetectedId] = useState(null);
  const [arucoDetectedWorldPose, setArucoDetectedWorldPose] = useState(null);
  const [savedPose, setSavedPose] = useState(null);
  const [savedPosesList, setSavedPosesList] = useState([]);
  const [waypoints, setWaypoints] = useState([]);
  const [activeWaypointIndex, setActiveWaypointIndex] = useState(-1);
  const [waypointAutoAdvanceEnabled, setWaypointAutoAdvanceEnabled] = useState(false);
  const [waypointAutoPublishEnabled, setWaypointAutoPublishEnabled] = useState(true);
  const [waypointDistanceThresholdInput, setWaypointDistanceThresholdInput] = useState("0.20");
  const [waypointCheckYaw, setWaypointCheckYaw] = useState(false);
  const [waypointYawThresholdInput, setWaypointYawThresholdInput] = useState("15");
  const [waypointInfo, setWaypointInfo] = useState("未開始");
  const [tracePoints, setTracePoints] = useState([]);
  const [traceRecording, setTraceRecording] = useState(false);
  const [traceReplayRunning, setTraceReplayRunning] = useState(false);
  const [traceReplayIndex, setTraceReplayIndex] = useState(-1);
  const [traceSampleMsInput, setTraceSampleMsInput] = useState("100");
  const [traceReplayMsInput, setTraceReplayMsInput] = useState("200");
  const [traceReplayLoop, setTraceReplayLoop] = useState(false);
  const [traceReplayAutoPublish, setTraceReplayAutoPublish] = useState(true);
  const [traceInfo, setTraceInfo] = useState("未開始");
  const [autoDriveCmdInfo, setAutoDriveCmdInfo] = useState("未送信");
  const [topicList, setTopicList] = useState([]);
  const [topicListLoading, setTopicListLoading] = useState(false);
  const [topicListError, setTopicListError] = useState("");
  const [selectedEchoTopic, setSelectedEchoTopic] = useState("");
  const [selectedEchoType, setSelectedEchoType] = useState("");
  const [topicEchoInfo, setTopicEchoInfo] = useState("未開始");
  const [topicEchoMessages, setTopicEchoMessages] = useState([]);
  const [topicEchoRunning, setTopicEchoRunning] = useState(false);
  const [cameraTopicInput, setCameraTopicInput] = useState("/camera/image_raw");
  const [cameraTopicName, setCameraTopicName] = useState("/camera/image_raw");
  const [cameraStreamRunning, setCameraStreamRunning] = useState(false);
  const [cameraStreamInfo, setCameraStreamInfo] = useState("未開始");
  const [cubeDebugTopicInput, setCubeDebugTopicInput] = useState("/cube_detection/debug_image");
  const [cubeDebugTopicName, setCubeDebugTopicName] = useState("/cube_detection/debug_image");
  const [cubeDebugStreamRunning, setCubeDebugStreamRunning] = useState(false);
  const [cubeDebugStreamInfo, setCubeDebugStreamInfo] = useState("未開始");
  const [cubeFrameUrl, setCubeFrameUrl] = useState("");
  const [cubeFrameMeta, setCubeFrameMeta] = useState({
    width: 0,
    height: 0,
    encoding: "",
    fps: 0,
  });
  const [cubeDetected, setCubeDetected] = useState(false);
  const [cubeUpdatedAt, setCubeUpdatedAt] = useState("");
  const [cubeInfo, setCubeInfo] = useState({
    cxNorm: 0,
    cyNorm: 0,
    widthNorm: 0,
    heightNorm: 0,
    depthM: 0,
    score: 0,
    areaPx: 0,
  });
  const [wallAngleTopicInput, setWallAngleTopicInput] = useState("/wall_detection/angle");
  const [wallAngleTopicName, setWallAngleTopicName] = useState("/wall_detection/angle");
  const [wallAngleRad, setWallAngleRad] = useState(0);
  const [wallAngleUpdatedAt, setWallAngleUpdatedAt] = useState("");
  const [wallFilteredPoints, setWallFilteredPoints] = useState([]);
  const [wallRansacParams, setWallRansacParams] = useState({ a: 0, b: 0, c: 0, inliers: 0, total: 0 });
  const [cameraFrameUrl, setCameraFrameUrl] = useState("");
  const [cameraFrameMeta, setCameraFrameMeta] = useState({
    width: 0,
    height: 0,
    encoding: "",
    fps: 0,
  });
  const [serialBridgePorts, setSerialBridgePorts] = useState([]);
  const [serialBridgeIds, setSerialBridgeIds] = useState([]);
  const [serialBridgeRunning, setSerialBridgeRunning] = useState(false);
  const [serialBridgePid, setSerialBridgePid] = useState("");
  const [serialBridgeInfo, setSerialBridgeInfo] = useState("未取得");
  const [serialBridgeLoading, setSerialBridgeLoading] = useState(false);
  const [serialBridgeLogs, setSerialBridgeLogs] = useState([]);
  const [serialBridgeLogLinesInput, setSerialBridgeLogLinesInput] = useState("200");
  const [serialBridgeLogLoading, setSerialBridgeLogLoading] = useState(false);
  const [serialBridgeLogRealtimeEnabled, setSerialBridgeLogRealtimeEnabled] = useState(false);
  const [yawOffsetDegInput, setYawOffsetDegInput] = useState("90");
  const [virtualOdomEnabled, setVirtualOdomEnabled] = useState(false);
  const [virtualOdomHzInput, setVirtualOdomHzInput] = useState("10");
  const virtualOdomEnabledRef = useRef(false);
  const arucoTargetIdValueRef = useRef(-1);
  const arucoCameraOffsetXRef = useRef(-0.1735);
  const arucoCameraOffsetYRef = useRef(0.0);
  const arucoDetectedIdRef = useRef(null);

  const backendBaseUrl = `${window.location.protocol}//${window.location.hostname}:3031`;

  const plannerCustomStateLabelMap = Object.fromEntries(
    plannerCustomStates.map((state) => [
      state.code,
      { ja: state.labelJa, en: state.labelEn },
    ])
  );

  const plannerAllStateCodes = Array.from(
    new Set([
      ...BUILTIN_PLANNER_STATE_CODES,
      ...plannerCustomStates.map((state) => state.code),
      ...plannerStateSequence,
    ])
  );

  const rosUrl = `${wsScheme}://${rosEndpoint.host}:${rosEndpoint.port}`;
  const tr = (jaText, enText) => getLocalizedText(language, jaText, enText);
  const localizedStatusText =
    status === "接続OK"
      ? tr("接続OK", "Connected")
      : status === "接続中..."
        ? tr("接続中...", "Connecting...")
        : status === "切断"
          ? tr("切断", "Disconnected")
          : status === "エラー"
            ? tr("エラー", "Error")
            : status;

  const translateRuntimeText = (text) => translateRuntimeByLanguage(language, text);

  const lockNoticeText = operationArmed
    ? tr("送信系機能が有効です。注意して操作してください。", "Sending features are enabled. Operate carefully.")
    : tr("ロック中: 送信系機能は無効化されています", "Locked: sending features are disabled");

  const renderLanguageSelect = (className, idPrefix) => (
    <label className={className}>
      <select
        className="lang-select-control"
        value={language}
        onChange={(e) => setLanguage(e.target.value)}
        aria-label={tr("言語選択", "Language")}
      >
        {LANGUAGE_OPTIONS.map((option) => (
          <option key={`${idPrefix}-${option.code}`} value={option.code}>
            {option.flag} {option.label}
          </option>
        ))}
      </select>
    </label>
  );

  const applyRosEndpoint = () => {
    const nextHost = rosHostInput.trim() || defaultRosHost;
    const nextPort = rosPortInput.trim() || "9090";

    setRosEndpoint({ host: nextHost, port: nextPort });
  };

  const applyJoyTopicName = () => {
    const nextTopic = joyTopicInput.trim() || "/joy_9";
    setJoyTopicName(nextTopic);
    console.log("Joy topic name updated to:", nextTopic);
  };

  const applyVirtualOdomTopicName = () => {
    const nextTopic = virtualOdomTopicInput.trim() || "odom_xy_yaw";
    setVirtualOdomTopicName(nextTopic);
    console.log("Virtual odometry topic updated to:", nextTopic);
  };

  const applyWallAngleTopicName = () => {
    const nextTopic = wallAngleTopicInput.trim() || "/wall_detection/angle";
    setWallAngleTopicName(nextTopic);
    console.log("Wall surface estimation topic updated to:", nextTopic);
  };

  const applyCubeDebugTopicName = () => {
    const nextTopic = cubeDebugTopicInput.trim() || "/cube_detection/debug_image";
    setCubeDebugTopicName(nextTopic);
    console.log("Cube debug topic updated to:", nextTopic);
  };

  const MAX_ACTIVE_PAGES = multiTabMode ? 2 : 1;

  const isPageActive = (page) => activePages.includes(page);

  const togglePage = (page) => {
    setActivePages((prev) => {
      if (!multiTabMode) {
        return prev[0] === page ? prev : [page];
      }
      if (prev.includes(page)) {
        return prev.length > 1 ? prev.filter((item) => item !== page) : prev;
      }
      if (prev.length >= MAX_ACTIVE_PAGES) {
        return prev;
      }
      return [...prev, page];
    });
  };

  const openOnlyPage = (page) => {
    setActivePages([page]);
  };

  useEffect(() => {
    if (!multiTabMode) {
      setActivePages((prev) => (prev.length > 0 ? [prev[0]] : ["controller"]));
    }
  }, [multiTabMode]);

  const getPageLabel = (page) => {
    if (page === "controller") return tr("コントローラ操作", "Controller");
    if (page === "sequence") return tr("シーケンス操作", "Sequence");
    if (page === "pose") return tr("座標・姿勢管理", "Pose");
    if (page === "planner") return tr("プランナー", "Planner");
    if (page === "wall-angle") return tr("壁面推定", "Wall Surface Estimation");
    if (page === "waypoint") return tr("ウェイポイント", "Waypoints");
    if (page === "teaching") return tr("ティーチング", "Teaching");
    if (page === "simulator") return tr("仮想オドメトリ", "Virtual Odom");
    if (page === "actuator") return tr("ダイレクト送信", "Actuator TX");
    if (page === "topic") return tr("トピック監視", "Topics");
    if (page === "camera") return tr("カメラ映像", "Camera");
    if (page === "cube-detection") return tr("立方体検知", "Cube Detection");
    if (page === "serial-bridge") return "Serial Bridge";
    if (page === "shutdown") return tr("強制停止", "Shutdown");
    if (page === "settings") return tr("設定", "Settings");
    return page;
  };

  const handlePageButtonClick = (page) => {
    const wasActive = isPageActive(page);
    togglePage(page);
    if (!wasActive && page === "serial-bridge") {
      refreshSerialBridgeStatus();
      refreshTopicList();
      refreshSerialBridgeLogs();
    }
  };

  const publishVirtualOdomNow = () => {
    if (!virtualOdomPubRef.current) return;
    virtualOdomPubRef.current.publish({
      data: [poseX, poseY, poseYaw],
    });
  };

  const resetVirtualOdomPose = () => {
    setPoseX(0);
    setPoseY(0);
    setPoseYaw(0);
  };

  const nudgeVirtualOdomPose = (dx = 0, dy = 0, dyawDeg = 0) => {
    setPoseX((prev) => Number((prev + dx).toFixed(3)));
    setPoseY((prev) => Number((prev + dy).toFixed(3)));
    setPoseYaw((prev) => normalizeYawRad(prev + (dyawDeg * Math.PI) / 180));
  };

  const renderVirtualOdomPanel = (panelId) => (
    <section className="serial-bridge-card" style={{ marginTop: 10 }}>
      <h3 className="serial-bridge-title">{tr("仮想オドメトリ発行", "Virtual Odometry Publisher")}</h3>
      <div className="serial-bridge-list virtual-odom-list">
        <button
          className={`toggle-button ${virtualOdomEnabled ? "toggle-on" : "toggle-off"}`}
          onClick={() => setVirtualOdomEnabled((prev) => !prev)}
        >
          {virtualOdomEnabled ? tr("定期発行: ON", "Periodic Publish: ON") : tr("定期発行: OFF", "Periodic Publish: OFF")}
        </button>

        <label className="serial-packet-label">
          {tr("オドメトリトピック", "Odometry Topic")}
          <input
            className="connection-input"
            value={virtualOdomTopicInput}
            onChange={(e) => setVirtualOdomTopicInput(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === "Enter") applyVirtualOdomTopicName();
            }}
            aria-label={`${panelId}-virtual-odom-topic`}
          />
        </label>
        <button className="connection-button btn-connect" onClick={applyVirtualOdomTopicName}>
          {tr("トピック適用", "Apply Topic")}
        </button>

        <label className="serial-packet-label">
          {tr("発行Hz", "Publish Hz")}
          <input
            className="connection-input"
            type="number"
            step="1"
            min="1"
            max="50"
            value={virtualOdomHzInput}
            onChange={(e) => setVirtualOdomHzInput(e.target.value)}
            disabled={!virtualOdomEnabled}
          />
        </label>

        <label className="serial-packet-label">
          {tr("発行X", "Publish X")}
          <input
            className="connection-input"
            type="number"
            step="0.1"
            value={poseX}
            onChange={(e) => setPoseX(parseFloatSafe(e.target.value, 0))}
          />
        </label>
        <label className="serial-packet-label">
          {tr("発行Y", "Publish Y")}
          <input
            className="connection-input"
            type="number"
            step="0.1"
            value={poseY}
            onChange={(e) => setPoseY(parseFloatSafe(e.target.value, 0))}
          />
        </label>
        <label className="serial-packet-label">
          {tr("発行Yaw (deg)", "Publish Yaw (deg)")}
          <input
            className="connection-input"
            type="number"
            step="1"
            value={(poseYaw * 180 / Math.PI).toFixed(1)}
            onChange={(e) => setPoseYaw(normalizeYawRad(parseFloatSafe(e.target.value, 0) * Math.PI / 180))}
          />
        </label>

        <button className="connection-button btn-send" onClick={publishVirtualOdomNow}>
          {tr("1回発行", "Publish Once")}
        </button>
        <button className="serial-clear-button" onClick={resetVirtualOdomPose}>
          {tr("座標姿勢リセット", "Reset Pose")}
        </button>

        <div className="virtual-odom-nudge-grid">
          <button className="connection-button btn-connect" onClick={() => nudgeVirtualOdomPose(0.1, 0, 0)}>
            {tr("X +0.1", "X +0.1")}
          </button>
          <button className="connection-button btn-connect" onClick={() => nudgeVirtualOdomPose(0, 0.1, 0)}>
            {tr("Y +0.1", "Y +0.1")}
          </button>
          <button className="connection-button btn-connect" onClick={() => nudgeVirtualOdomPose(0, 0, 5)}>
            {tr("Yaw +5°", "Yaw +5deg")}
          </button>
          <button className="connection-button btn-connect" onClick={() => nudgeVirtualOdomPose(-0.1, 0, 0)}>
            {tr("X -0.1", "X -0.1")}
          </button>
          <button className="connection-button btn-connect" onClick={() => nudgeVirtualOdomPose(0, -0.1, 0)}>
            {tr("Y -0.1", "Y -0.1")}
          </button>
          <button className="connection-button btn-connect" onClick={() => nudgeVirtualOdomPose(0, 0, -5)}>
            {tr("Yaw -5°", "Yaw -5deg")}
          </button>
        </div>

        <p className="connection-hint">
          {tr("発行先トピック", "Publish topic")}: {virtualOdomTopicName}
        </p>
        <p className="connection-hint">
          {tr("発行値", "Publish values")}: X={poseX.toFixed(3)}, Y={poseY.toFixed(3)}, Yaw={(poseYaw * 180 / Math.PI).toFixed(1)}°
        </p>
      </div>
    </section>
  );

  const publishPlannerState = (stateCode) => {
    if (!taskStateCommandRef.current) return;
    taskStateCommandRef.current.publish({ data: Number(stateCode) });
  };

  const publishPlannerColor = (colorCode) => {
    if (!taskColorCommandRef.current) return;
    taskColorCommandRef.current.publish({ data: Number(colorCode) });
  };

  const publishPlannerCell = () => {
    if (!taskCellCommandRef.current) return;
    const nextCell = Number.parseInt(plannerCellInput, 10);
    taskCellCommandRef.current.publish({ data: Number.isFinite(nextCell) ? nextCell : 0 });
  };

  const publishPlannerCellValue = (cellValue) => {
    if (!taskCellCommandRef.current) return;
    taskCellCommandRef.current.publish({ data: Number(cellValue) });
  };

  const publishPlannerTransitionMode = (modeCode) => {
    if (!taskTransitionModeRef.current) return;
    taskTransitionModeRef.current.publish({ data: Number(modeCode) });
  };

  const publishPlannerAutoSendEnabled = (enabled) => {
    if (!taskAutoSendEnabledRef.current) return;
    taskAutoSendEnabledRef.current.publish({ data: Boolean(enabled) });
    setPlannerAutoSendEnabled(Boolean(enabled));
  };

  const publishPlannerStateSequence = (sequence = plannerStateSequence) => {
    if (!taskStateSequenceRef.current) return;
    taskStateSequenceRef.current.publish({ data: sequence.map((value) => Number(value)) });
  };

  const publishPlannerStatePose = (stateCode) => {
    if (!taskStatePoseRef.current) return;
    const config = plannerStatePoseConfig[stateCode];
    if (!config) return;

    const xValue = Number.parseFloat(config.x);
    const yValue = Number.parseFloat(config.y);
    const yawValue = Number.parseFloat(config.yaw);

    taskStatePoseRef.current.publish({
      data: [
        Number(stateCode),
        Number.isFinite(xValue) ? xValue : 0,
        Number.isFinite(yValue) ? yValue : 0,
        Number.isFinite(yawValue) ? yawValue : 0,
        config.enabled ? 1 : 0,
      ],
    });
  };

  const publishPlannerStateMode = (stateCode) => {
    if (!taskStateModeRef.current) return;
    const config = plannerStateModeConfig[stateCode];
    if (!config) return;

    taskStateModeRef.current.publish({
      data: [Number(stateCode), config.enabled ? Number(config.modeCode) : -1],
    });
  };

  const publishPlannerStateOdomReset = (stateCode) => {
    if (!taskStateOdomResetRef.current) return;
    const config = plannerStateOdomResetConfig[stateCode];
    if (!config) return;

    taskStateOdomResetRef.current.publish({
      data: [Number(stateCode), config.enabled ? 1 : 0],
    });
  };

  const movePlannerStateSequence = (stateCode, direction) => {
    setPlannerStateSequence((prevSequence) => {
      const currentIndex = prevSequence.indexOf(stateCode);
      if (currentIndex < 0) {
        return prevSequence;
      }

      const nextIndex = currentIndex + direction;
      if (nextIndex < 0 || nextIndex >= prevSequence.length) {
        return prevSequence;
      }

      const nextSequence = [...prevSequence];
      [nextSequence[currentIndex], nextSequence[nextIndex]] = [nextSequence[nextIndex], nextSequence[currentIndex]];
      return nextSequence;
    });
  };

  const resetPlannerStateSequence = () => {
    setPlannerStateSequence([
      ...BUILTIN_PLANNER_STATE_CODES,
      ...plannerCustomStates.map((state) => state.code),
    ]);
  };

  const addPlannerCustomState = () => {
    const code = Number.parseInt(plannerNewStateCodeInput, 10);
    const labelJa = plannerNewStateJaInput.trim();
    const labelEn = plannerNewStateEnInput.trim();

    if (!Number.isFinite(code) || code < 6) {
      setPlannerStatusText(tr("状態コードは 6 以上の整数を指定してください", "State code must be an integer >= 6"));
      return;
    }

    if (BUILTIN_PLANNER_STATE_CODES.includes(code) || plannerCustomStates.some((state) => state.code === code)) {
      setPlannerStatusText(tr(`状態コード ${code} は既に存在します`, `State code ${code} already exists`));
      return;
    }

    if (!labelJa || !labelEn) {
      setPlannerStatusText(tr("日本語名と英語名を入力してください", "Please enter both Japanese and English labels"));
      return;
    }

    const nextCustomStates = [...plannerCustomStates, { code, labelJa, labelEn }].sort((a, b) => a.code - b.code);
    setPlannerCustomStates(nextCustomStates);
    setPlannerStateSequence((prev) => (prev.includes(code) ? prev : [...prev, code]));
    setPlannerStatePoseConfig((prev) => ({
      ...prev,
      [code]: { enabled: false, x: "0.0", y: "0.0", yaw: "0.0" },
    }));
    setPlannerStateModeConfig((prev) => ({
      ...prev,
      [code]: { enabled: false, modeCode: 1 },
    }));
    setPlannerStateOdomResetConfig((prev) => ({
      ...prev,
      [code]: { enabled: false },
    }));
    setPlannerNewStateCodeInput("");
    setPlannerNewStateJaInput("");
    setPlannerNewStateEnInput("");
    setPlannerStatusText(tr(`状態を追加しました: ${code}`, `State added: ${code}`));
  };

  const removePlannerCustomState = (code) => {
    setPlannerCustomStates((prev) => prev.filter((state) => state.code !== code));
    setPlannerStateSequence((prev) => prev.filter((stateCode) => stateCode !== code));
    setPlannerStatePoseConfig((prev) => {
      const next = { ...prev };
      delete next[code];
      return next;
    });
    setPlannerStateModeConfig((prev) => {
      const next = { ...prev };
      delete next[code];
      return next;
    });
    setPlannerStateOdomResetConfig((prev) => {
      const next = { ...prev };
      delete next[code];
      return next;
    });
    setPlannerStatusText(tr(`状態を削除しました: ${code}`, `State removed: ${code}`));
  };

  const applyPlannerStateConfigData = (parsed, { source = "import" } = {}) => {
    const importedCustomStatesRaw = Array.isArray(parsed?.customStates) ? parsed.customStates : [];
    const normalizedCustomStates = [];
    importedCustomStatesRaw.forEach((item) => {
      const code = Number.parseInt(item?.code, 10);
      if (!Number.isFinite(code) || code < 6) {
        return;
      }
      if (BUILTIN_PLANNER_STATE_CODES.includes(code)) {
        return;
      }
      if (normalizedCustomStates.some((state) => state.code === code)) {
        return;
      }
      const labelJa = String(item?.labelJa || item?.ja || `状態 ${code}`).trim();
      const labelEn = String(item?.labelEn || item?.en || `State ${code}`).trim();
      normalizedCustomStates.push({
        code,
        labelJa: labelJa || `状態 ${code}`,
        labelEn: labelEn || `State ${code}`,
      });
    });

    normalizedCustomStates.sort((a, b) => a.code - b.code);
    const validCodes = Array.from(new Set([
      ...BUILTIN_PLANNER_STATE_CODES,
      ...normalizedCustomStates.map((state) => state.code),
    ]));
    const validStateSet = new Set(validCodes);

    const rawSequence = Array.isArray(parsed?.stateSequence) ? parsed.stateSequence : [];
    const normalizedSequence = [];
    rawSequence.forEach((value) => {
      const code = Number(value);
      if (!Number.isFinite(code) || !validStateSet.has(code)) {
        return;
      }
      if (!normalizedSequence.includes(code)) {
        normalizedSequence.push(code);
      }
    });
    validCodes.forEach((code) => {
      if (!normalizedSequence.includes(code)) {
        normalizedSequence.push(code);
      }
    });

    const defaultPoseConfig = createPlannerStatePoseConfig(validCodes);
    const defaultModeConfig = createPlannerStateModeConfig(validCodes);
    const defaultOdomResetConfig = createPlannerStateOdomResetConfig(validCodes);
    const importedPose = parsed?.statePoseConfig && typeof parsed.statePoseConfig === "object" ? parsed.statePoseConfig : {};
    const importedMode = parsed?.stateModeConfig && typeof parsed.stateModeConfig === "object" ? parsed.stateModeConfig : {};
    const importedOdomReset = parsed?.stateOdomResetConfig && typeof parsed.stateOdomResetConfig === "object" ? parsed.stateOdomResetConfig : {};

    const nextPoseConfig = Object.fromEntries(validCodes.map((code) => {
      const fallback = defaultPoseConfig[code];
      const raw = importedPose[String(code)] ?? importedPose[code] ?? {};
      const x = Number(raw?.x);
      const y = Number(raw?.y);
      const yaw = Number(raw?.yaw);
      return [code, {
        enabled: Boolean(raw?.enabled),
        x: Number.isFinite(x) ? String(x) : fallback.x,
        y: Number.isFinite(y) ? String(y) : fallback.y,
        yaw: Number.isFinite(yaw) ? String(yaw) : fallback.yaw,
      }];
    }));

    const nextModeConfig = Object.fromEntries(validCodes.map((code) => {
      const fallback = defaultModeConfig[code];
      const raw = importedMode[String(code)] ?? importedMode[code] ?? {};
      const modeCode = Number(raw?.modeCode);
      return [code, {
        enabled: Boolean(raw?.enabled),
        modeCode: Number.isFinite(modeCode) && modeCode >= 0 && modeCode <= 2 ? modeCode : fallback.modeCode,
      }];
    }));

    const nextOdomResetConfig = Object.fromEntries(validCodes.map((code) => {
      const raw = importedOdomReset[String(code)] ?? importedOdomReset[code] ?? {};
      return [code, {
        enabled: Boolean(raw?.enabled),
      }];
    }));

    setPlannerCustomStates(normalizedCustomStates);
    setPlannerStateSequence(normalizedSequence);
    setPlannerStatePoseConfig(nextPoseConfig);
    setPlannerStateModeConfig(nextModeConfig);
    setPlannerStateOdomResetConfig(nextOdomResetConfig);
    setPlannerConfigIsDirty(false);
    setPlannerStatusText(
      source === "startup"
        ? tr(`最新設定を自動読込しました (${normalizedSequence.length} 状態)`, `Loaded latest config automatically (${normalizedSequence.length} states)`)
        : tr(`状態設定をインポートしました (${normalizedSequence.length} 状態)`, `State configuration imported (${normalizedSequence.length} states)`)
    );
  };

  const exportPlannerStateConfig = async () => {
    const allStateCodes = [...plannerAllStateCodes];
    const exportData = {
      format: "nr26-planner-state-config",
      version: 1,
      exportedAt: new Date().toISOString(),
      customStates: plannerCustomStates.map((state) => ({
        code: Number(state.code),
        labelJa: state.labelJa,
        labelEn: state.labelEn,
      })),
      stateSequence: plannerStateSequence.map((code) => Number(code)),
      statePoseConfig: Object.fromEntries(
        allStateCodes.map((code) => {
          const config = plannerStatePoseConfig[code] || createPlannerStatePoseConfig()[code] || {
            enabled: false,
            x: "0.0",
            y: "0.0",
            yaw: "0.0",
          };
          return [code, {
            enabled: Boolean(config.enabled),
            x: String(config.x ?? "0.0"),
            y: String(config.y ?? "0.0"),
            yaw: String(config.yaw ?? "0.0"),
          }];
        })
      ),
      stateModeConfig: Object.fromEntries(
        allStateCodes.map((code) => {
          const config = plannerStateModeConfig[code] || createPlannerStateModeConfig()[code] || {
            enabled: false,
            modeCode: 1,
          };
          return [code, {
            enabled: Boolean(config.enabled),
            modeCode: Number(config.modeCode),
          }];
        })
      ),
      stateOdomResetConfig: Object.fromEntries(
        allStateCodes.map((code) => {
          const config = plannerStateOdomResetConfig[code] || createPlannerStateOdomResetConfig()[code] || {
            enabled: false,
          };
          return [code, {
            enabled: Boolean(config.enabled),
          }];
        })
      ),
    };

    const json = JSON.stringify(exportData, null, 2);
    const blob = new Blob([json], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    const stamp = new Date().toISOString().replace(/[:.]/g, "-");
    link.href = url;
    link.download = `planner_state_config_${stamp}.json`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);

    const savedResult = await saveJsonExportToAppDirectory("planner_state_config", exportData);
    const saveSuffix = savedResult.ok
      ? tr(` / 保存: ${savedResult.fileName}`, ` / saved: ${savedResult.fileName}`)
      : tr(" / App.jsディレクトリ保存失敗", " / failed to save into App.js directory");

    setPlannerStatusText(
      tr(
        `状態設定をエクスポートしました${saveSuffix}`,
        `State configuration exported${saveSuffix}`
      )
    );
    setPlannerConfigIsDirty(false);
  };

  const importPlannerStateConfigFromFile = async (event) => {
    const file = event.target.files?.[0];
    event.target.value = "";

    if (!file) {
      return;
    }

    try {
      const text = await file.text();
      const parsed = JSON.parse(text);
      applyPlannerStateConfigData(parsed, { source: "import" });
    } catch (error) {
      console.error("Failed to import planner state configuration:", error);
      setPlannerStatusText(
        tr(
          "状態設定のインポートに失敗しました (JSON形式を確認してください)",
          "Failed to import state configuration (check JSON format)"
        )
      );
    }
  };

  const loadLatestPlannerStateConfigFromBackend = async () => {
    try {
      const response = await fetch(`${backendBaseUrl}/api/json-exports/latest?category=planner_state_config`);
      if (!response.ok) {
        return;
      }
      const payload = await response.json();
      if (!payload?.found || !payload?.data) {
        return;
      }
      applyPlannerStateConfigData(payload.data, { source: "startup" });
    } catch (error) {
      console.warn("Failed to auto-load latest planner state configuration:", error);
    }
  };

  const loadPlannerExportsList = async () => {
    try {
      const response = await fetch(`${backendBaseUrl}/api/json-exports/list?category=planner_state_config`);
      if (!response.ok) {
        setPlannerExportsList([]);
        return;
      }
      const payload = await response.json();
      const files = Array.isArray(payload?.files) ? payload.files : [];
      setPlannerExportsList(files);
    } catch (error) {
      console.warn("Failed to load planner exports list:", error);
      setPlannerExportsList([]);
    }
  };

  const loadPlannerExportFile = async (relativePath) => {
    try {
      const response = await fetch(
        `${backendBaseUrl}/api/json-exports/get?category=planner_state_config&path=${encodeURIComponent(relativePath)}`
      );
      if (!response.ok) {
        setPlannerStatusText(tr("ファイル読込に失敗しました", "Failed to load file"));
        return;
      }
      const payload = await response.json();
      if (!payload?.found || !payload?.data) {
        setPlannerStatusText(tr("ファイルが見つかりません", "File not found"));
        return;
      }
      applyPlannerStateConfigData(payload.data, { source: "import" });
      setPlannerSelectedExportPath("");
    } catch (error) {
      console.warn("Failed to load planner export file:", error);
      setPlannerStatusText(tr("ファイル読込に失敗しました", "Failed to load file"));
    }
  };

  useEffect(() => {
    loadLatestPlannerStateConfigFromBackend();
  }, [backendBaseUrl]);

  const updatePlannerStatePose = (stateCode, key, value) => {
    setPlannerStatePoseConfig((prev) => ({
      ...prev,
      [stateCode]: {
        ...prev[stateCode],
        [key]: value,
      },
    }));
  };

  const updatePlannerStateMode = (stateCode, nextModeCode) => {
    setPlannerStateModeConfig((prev) => ({
      ...prev,
      [stateCode]: {
        enabled: nextModeCode >= 0,
        modeCode: nextModeCode >= 0 ? nextModeCode : prev[stateCode]?.modeCode ?? 1,
      },
    }));
  };

  const updatePlannerStateOdomReset = (stateCode, enabled) => {
    setPlannerStateOdomResetConfig((prev) => ({
      ...prev,
      [stateCode]: {
        enabled: Boolean(enabled),
      },
    }));
  };

  const togglePlannerTransitionMode = () => {
    const nextMode = plannerTransitionModeCode === 1 ? 0 : 1;
    publishPlannerTransitionMode(nextMode);
  };

  const plannerStateLabel = getPlannerStateLabel(plannerStateCode, language, plannerCustomStateLabelMap);
  const plannerColorLabel = getPlannerColorLabel(plannerColorCode, language);
  const plannerModeLabel = getPlannerModeLabel(plannerTransitionModeCode, language);
  const plannerLayoutRed = getMffLayout(1);
  const plannerLayoutBlue = getMffLayout(0);
  const showRedCourt = plannerColorCode !== 0;
  const showBlueCourt = plannerColorCode !== 1;
  const isSingleCourtView = (showRedCourt ? 1 : 0) + (showBlueCourt ? 1 : 0) === 1;

  const callRosService = (serviceRef, requestData) =>
    new Promise((resolve, reject) => {
      if (!serviceRef.current) {
        reject(new Error("サービスが初期化されていません"));
        return;
      }

      const request = requestData || {};
      serviceRef.current.callService(
        request,
        (response) => resolve(response),
        (error) => reject(error || new Error("サービス呼び出しに失敗しました"))
      );
    });

  const stopTopicEcho = () => {
    if (topicEchoSubRef.current) {
      try {
        topicEchoSubRef.current.unsubscribe();
      } catch (error) {
        console.warn("Error unsubscribing topic echo:", error);
      }
      topicEchoSubRef.current = null;
    }

    setTopicEchoRunning(false);
  };

  const stopCameraStream = () => {
    if (cameraSubRef.current) {
      try {
        cameraSubRef.current.unsubscribe?.();
      } catch (error) {
        console.warn("Error unsubscribing camera topic:", error);
      }
      cameraSubRef.current = null;
    }
    setCameraStreamRunning(false);
  };

  const applyCameraTopicPreset = (topicName) => {
    setCameraTopicInput(topicName);
    setCameraTopicName(topicName);
    setCameraStreamInfo(`トピック設定: ${topicName}`);
  };

  const stopCubeDebugStream = () => {
    if (cubeDebugSubRef.current) {
      try {
        cubeDebugSubRef.current.unsubscribe?.();
      } catch (error) {
        console.warn("Error unsubscribing cube debug topic:", error);
      }
      cubeDebugSubRef.current = null;
    }
    setCubeDebugStreamRunning(false);
  };

  const decodeRosUint8Array = (dataField) => {
    if (Array.isArray(dataField)) {
      return Uint8Array.from(dataField.map((v) => Number(v) & 0xff));
    }
    if (typeof dataField === "string") {
      try {
        const bin = atob(dataField);
        const out = new Uint8Array(bin.length);
        for (let i = 0; i < bin.length; i += 1) {
          out[i] = bin.charCodeAt(i);
        }
        return out;
      } catch (error) {
        console.warn("Failed to decode base64 image data:", error);
      }
    }
    return null;
  };

  const toImageDataFromRosImage = (msg) => {
    const width = Number(msg?.width) || 0;
    const height = Number(msg?.height) || 0;
    const encoding = (msg?.encoding || "").toLowerCase();
    const raw = decodeRosUint8Array(msg?.data);
    if (!width || !height || !raw) {
      return null;
    }

    const rgba = new Uint8ClampedArray(width * height * 4);

    if (encoding === "mono8") {
      const pixels = Math.min(width * height, raw.length);
      for (let i = 0; i < pixels; i += 1) {
        const v = raw[i];
        const j = i * 4;
        rgba[j] = v;
        rgba[j + 1] = v;
        rgba[j + 2] = v;
        rgba[j + 3] = 255;
      }
      return { width, height, encoding, imageData: new ImageData(rgba, width, height) };
    }

    if (encoding === "16uc1") {
      const pixels = Math.min(width * height, Math.floor(raw.length / 2));
      for (let i = 0; i < pixels; i += 1) {
        const depthMm = raw[i * 2] | (raw[i * 2 + 1] << 8);
        const normalized = Math.max(0, Math.min(255, Math.round((Math.min(depthMm, 4000) / 4000) * 255)));
        const j = i * 4;
        rgba[j] = normalized;
        rgba[j + 1] = normalized;
        rgba[j + 2] = normalized;
        rgba[j + 3] = 255;
      }
      return { width, height, encoding, imageData: new ImageData(rgba, width, height) };
    }

    if (encoding === "rgb8" || encoding === "bgr8") {
      const pixels = Math.min(width * height, Math.floor(raw.length / 3));
      const isBgr = encoding === "bgr8";
      for (let i = 0; i < pixels; i += 1) {
        const src = i * 3;
        const j = i * 4;
        const r = isBgr ? raw[src + 2] : raw[src];
        const g = raw[src + 1];
        const b = isBgr ? raw[src] : raw[src + 2];
        rgba[j] = r;
        rgba[j + 1] = g;
        rgba[j + 2] = b;
        rgba[j + 3] = 255;
      }
      return { width, height, encoding, imageData: new ImageData(rgba, width, height) };
    }

    if (encoding === "rgba8" || encoding === "bgra8") {
      const pixels = Math.min(width * height, Math.floor(raw.length / 4));
      const isBgra = encoding === "bgra8";
      for (let i = 0; i < pixels; i += 1) {
        const src = i * 4;
        const j = i * 4;
        const r = isBgra ? raw[src + 2] : raw[src];
        const g = raw[src + 1];
        const b = isBgra ? raw[src] : raw[src + 2];
        const a = raw[src + 3];
        rgba[j] = r;
        rgba[j + 1] = g;
        rgba[j + 2] = b;
        rgba[j + 3] = a;
      }
      return { width, height, encoding, imageData: new ImageData(rgba, width, height) };
    }

    return { width, height, encoding, imageData: null };
  };

  const startCameraStream = () => {
    const topicName = cameraTopicInput.trim();
    if (!topicName) {
      setCameraStreamInfo("トピック名を入力してください");
      return;
    }
    if (!rosRef.current) {
      setCameraStreamInfo("ROS未接続のため開始できません");
      return;
    }

    stopCameraStream();
    setCameraTopicName(topicName);

    const cameraTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topicName,
      messageType: "sensor_msgs/Image",
    });

    let frameCount = 0;
    let lastFpsAt = performance.now();
    let fps = 0;

    cameraTopic.subscribe((msg) => {
      const converted = toImageDataFromRosImage(msg);
      if (!converted) {
        setCameraStreamInfo("画像データのデコードに失敗しました");
        return;
      }

      if (!converted.imageData) {
        setCameraFrameMeta((prev) => ({
          ...prev,
          width: converted.width,
          height: converted.height,
          encoding: converted.encoding,
        }));
        setCameraStreamInfo(`未対応エンコーディング: ${converted.encoding || "unknown"}`);
        return;
      }

      const canvas = document.createElement("canvas");
      canvas.width = converted.width;
      canvas.height = converted.height;
      const ctx = canvas.getContext("2d");
      if (!ctx) {
        setCameraStreamInfo("Canvas初期化に失敗しました");
        return;
      }

      ctx.putImageData(converted.imageData, 0, 0);
      const url = canvas.toDataURL("image/png");
      setCameraFrameUrl(url);

      frameCount += 1;
      const now = performance.now();
      if (now - lastFpsAt >= 1000) {
        fps = Math.round((frameCount * 1000) / (now - lastFpsAt));
        frameCount = 0;
        lastFpsAt = now;
      }

      setCameraFrameMeta({
        width: converted.width,
        height: converted.height,
        encoding: converted.encoding,
        fps,
      });
      setCameraStreamInfo("ストリーミング中");
    });

    cameraSubRef.current = cameraTopic;
    setCameraStreamRunning(true);
    setCameraStreamInfo(`購読開始: ${topicName}`);
  };

  const startCubeDebugStream = () => {
    const topicName = cubeDebugTopicInput.trim();
    if (!topicName) {
      setCubeDebugStreamInfo("トピック名を入力してください");
      return;
    }
    if (!rosRef.current) {
      setCubeDebugStreamInfo("ROS未接続のため開始できません");
      return;
    }

    stopCubeDebugStream();
    setCubeDebugTopicName(topicName);

    const cubeTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topicName,
      messageType: "sensor_msgs/Image",
    });

    let frameCount = 0;
    let lastFpsAt = performance.now();
    let fps = 0;

    cubeTopic.subscribe((msg) => {
      const converted = toImageDataFromRosImage(msg);
      if (!converted) {
        setCubeDebugStreamInfo("画像データのデコードに失敗しました");
        return;
      }

      if (!converted.imageData) {
        setCubeFrameMeta((prev) => ({
          ...prev,
          width: converted.width,
          height: converted.height,
          encoding: converted.encoding,
        }));
        setCubeDebugStreamInfo(`未対応エンコーディング: ${converted.encoding || "unknown"}`);
        return;
      }

      const canvas = document.createElement("canvas");
      canvas.width = converted.width;
      canvas.height = converted.height;
      const ctx = canvas.getContext("2d");
      if (!ctx) {
        setCubeDebugStreamInfo("Canvas初期化に失敗しました");
        return;
      }

      ctx.putImageData(converted.imageData, 0, 0);
      setCubeFrameUrl(canvas.toDataURL("image/png"));

      frameCount += 1;
      const now = performance.now();
      if (now - lastFpsAt >= 1000) {
        fps = Math.round((frameCount * 1000) / (now - lastFpsAt));
        frameCount = 0;
        lastFpsAt = now;
      }

      setCubeFrameMeta({
        width: converted.width,
        height: converted.height,
        encoding: converted.encoding,
        fps,
      });
      setCubeDebugStreamInfo("ストリーミング中");
    });

    cubeDebugSubRef.current = cubeTopic;
    setCubeDebugStreamRunning(true);
    setCubeDebugStreamInfo(`購読開始: ${topicName}`);
  };

  const extractSerialBridgeIds = (topics) => {
    const idSet = new Set();
    topics.forEach((item) => {
      const name = item?.name || "";
      const match = name.match(/^serial_(?:rx|tx)_(\d+)$/);
      if (match) {
        idSet.add(Number.parseInt(match[1], 10));
      }
    });

    return Array.from(idSet).sort((a, b) => a - b);
  };

  const addTimestampToLogLines = (lines) => {
    const fetchedAt = new Date().toLocaleTimeString(language === "ja" ? "ja-JP" : "en-US");
    return lines.map((line) => {
      if (/^\[\d{1,2}:\d{2}:\d{2}\]/.test(line)) {
        return line;
      }
      return `[${fetchedAt}] ${line}`;
    });
  };

  const parsedLogLineLimit = Number.parseInt(serialBridgeLogLinesInput, 10);
  const serialBridgeLogLineLimit = Number.isFinite(parsedLogLineLimit)
    ? Math.max(10, Math.min(1000, parsedLogLineLimit))
    : 200;

  const refreshTopicList = async () => {
    if (!rosRef.current || !rosTopicsServiceRef.current) {
      setTopicListError("ROS未接続のため取得できません");
      return [];
    }

    setTopicListLoading(true);
    setTopicListError("");
    try {
      const response = await callRosService(rosTopicsServiceRef, {});
      const topics = Array.isArray(response?.topics) ? response.topics : [];
      const types = Array.isArray(response?.types) ? response.types : [];
      const mapped = topics.map((name, index) => ({
        name,
        type: types[index] || "",
      }));

      mapped.sort((a, b) => a.name.localeCompare(b.name));
      setTopicList(mapped);
      setSerialBridgeIds(extractSerialBridgeIds(mapped));

      if (mapped.length === 0) {
        setTopicListError("トピックが見つかりません");
      }
      return mapped;
    } catch (error) {
      console.error("Failed to fetch topic list:", error);
      setTopicListError("トピック一覧の取得に失敗しました");
      return [];
    } finally {
      setTopicListLoading(false);
    }
  };

  const refreshSerialBridgeStatus = async () => {
    setSerialBridgeLoading(true);
    try {
      const response = await fetch(`${backendBaseUrl}/api/serial-bridge/status`);
      if (!response.ok) {
        throw new Error(`status ${response.status}`);
      }
      const data = await response.json();
      setSerialBridgePorts(Array.isArray(data?.ports) ? data.ports : []);
      setSerialBridgeRunning(Boolean(data?.running));
      setSerialBridgePid(data?.pid ? String(data.pid) : "");
      setSerialBridgeInfo("状態を更新しました");
    } catch (error) {
      console.error("Failed to fetch serial bridge status:", error);
      setSerialBridgeInfo("状態取得に失敗しました (backend未起動の可能性)");
      setSerialBridgeRunning(false);
      setSerialBridgePid("");
      setSerialBridgePorts([]);
    } finally {
      setSerialBridgeLoading(false);
    }
  };

  const refreshSerialBridgeLogs = async () => {
    setSerialBridgeLogLoading(true);
    try {
      const response = await fetch(`${backendBaseUrl}/api/serial-bridge/logs?lines=${serialBridgeLogLineLimit}`);
      if (!response.ok) {
        throw new Error(`status ${response.status}`);
      }
      const data = await response.json();
      const lines = Array.isArray(data?.lines) ? data.lines : [];
      setSerialBridgeLogs(addTimestampToLogLines(lines));
    } catch (error) {
      console.error("Failed to fetch serial bridge logs:", error);
      setSerialBridgeLogs(["ログ取得に失敗しました"]);
    } finally {
      setSerialBridgeLogLoading(false);
    }
  };

  const startSerialBridgeFromConsole = async () => {
    setSerialBridgeLoading(true);
    try {
      const response = await fetch(`${backendBaseUrl}/api/serial-bridge/start`, {
        method: "POST",
      });
      if (!response.ok) {
        throw new Error(`status ${response.status}`);
      }
      const data = await response.json();
      setSerialBridgeRunning(Boolean(data?.running));
      setSerialBridgePid(data?.pid ? String(data.pid) : "");
      setSerialBridgeInfo(data?.message || "serial_bridge を起動しました");
      await refreshSerialBridgeStatus();
      await refreshTopicList();
      await refreshSerialBridgeLogs();
    } catch (error) {
      console.error("Failed to start serial bridge:", error);
      setSerialBridgeInfo("serial_bridge の起動に失敗しました");
    } finally {
      setSerialBridgeLoading(false);
    }
  };

  const applySettingsValues = async () => {
    applyRosEndpoint();
    applyJoyTopicName();
    await refreshSerialBridgeLogs();
    setSerialBridgeInfo("設定を適用しました");
  };

  const stopSerialBridgeFromConsole = async () => {
    setSerialBridgeLoading(true);
    try {
      const response = await fetch(`${backendBaseUrl}/api/serial-bridge/stop`, {
        method: "POST",
      });
      if (!response.ok) {
        throw new Error(`status ${response.status}`);
      }
      const data = await response.json();
      setSerialBridgeRunning(Boolean(data?.running));
      setSerialBridgeInfo(data?.message || "serial_bridge を停止しました");
      await refreshSerialBridgeStatus();
      await refreshTopicList();
      await refreshSerialBridgeLogs();
    } catch (error) {
      console.error("Failed to stop serial bridge:", error);
      setSerialBridgeInfo("serial_bridge の停止に失敗しました");
    } finally {
      setSerialBridgeLoading(false);
    }
  };

  const forceShutdownBackendFromConsole = async () => {
    const confirmed = window.confirm(
      tr(
        "console backend を強制シャットダウンします。\nこの後、状態取得や起動/停止機能は再起動まで使えません。続行しますか？",
        "Force shutdown console backend?\nStatus/start/stop features will be unavailable until restart. Continue?"
      )
    );
    if (!confirmed) {
      return;
    }

    setSerialBridgeLoading(true);
    try {
      const response = await fetch(`${backendBaseUrl}/api/backend/force-shutdown`, {
        method: "POST",
      });
      if (!response.ok) {
        throw new Error(`status ${response.status}`);
      }
      const data = await response.json();
      setSerialBridgeInfo(data?.message || "console backend を強制シャットダウンしました");
      setSerialBridgeRunning(false);
      setSerialBridgePid("");
      setSerialBridgePorts([]);
    } catch (error) {
      console.error("Failed to force shutdown backend:", error);
      setSerialBridgeInfo("console backend の強制シャットダウンに失敗しました");
    } finally {
      setSerialBridgeLoading(false);
    }
  };

  const forceShutdownFrontendFromConsole = () => {
    const confirmed = window.confirm(
      tr(
        "フロントエンドを強制シャットダウンします。\n画面操作は停止し、再読み込みまで復帰できません。続行しますか？",
        "Force shutdown frontend?\nUI will stop and cannot recover until reload. Continue?"
      )
    );
    if (!confirmed) {
      return;
    }

    setOperationArmed(false);
    setControllerEnabled(false);
    setSerialPeriodicEnabled(false);
    setSerialBridgeLogRealtimeEnabled(false);
    setTraceRecording(false);
    setTraceReplayRunning(false);
    openOnlyPage("shutdown");
    resetAllControls();
    stopTopicEcho();
    if (rosRef.current) {
      rosRef.current.close();
    }
    setStatus("切断");
    setFrontendForceStopped(true);
  };

  useEffect(() => {
    if (!serialBridgeLogRealtimeEnabled || !isPageActive("serial-bridge")) {
      return undefined;
    }

    const pollLogs = async () => {
      setSerialBridgeLogLoading(true);
      try {
        const response = await fetch(`${backendBaseUrl}/api/serial-bridge/logs?lines=${serialBridgeLogLineLimit}`);
        if (!response.ok) {
          throw new Error(`status ${response.status}`);
        }
        const data = await response.json();
        const lines = Array.isArray(data?.lines) ? data.lines : [];
        setSerialBridgeLogs(addTimestampToLogLines(lines));
      } catch (error) {
        console.error("Failed to fetch serial bridge logs:", error);
        setSerialBridgeLogs(["ログ取得に失敗しました"]);
      } finally {
        setSerialBridgeLogLoading(false);
      }
    };

    const timer = setInterval(() => {
      pollLogs();
    }, 1000);

    pollLogs();

    return () => {
      clearInterval(timer);
    };
  }, [serialBridgeLogRealtimeEnabled, activePages, backendBaseUrl, serialBridgeLogLineLimit]);

  useEffect(() => {
    if (!isPageActive("serial-bridge")) {
      return;
    }

    const logBox = serialBridgeLogBoxRef.current;
    if (!logBox) {
      return;
    }

    logBox.scrollTop = logBox.scrollHeight;
  }, [serialBridgeLogs, serialBridgeLogLoading, activePages]);

  const startTopicEcho = async () => {
    const topicName = selectedEchoTopic.trim();
    if (!topicName) {
      setTopicEchoInfo("トピック名を選択してください");
      return;
    }

    if (!rosRef.current) {
      setTopicEchoInfo("ROS未接続のため開始できません");
      return;
    }

    let topicType = selectedEchoType;
    try {
      if (!topicType) {
        const response = await callRosService(rosTopicTypeServiceRef, { topic: topicName });
        topicType = response?.type || "";
      }
    } catch (error) {
      console.error("Failed to resolve topic type:", error);
    }

    if (!topicType) {
      setTopicEchoInfo("トピック型の取得に失敗しました");
      return;
    }

    stopTopicEcho();
    setTopicEchoMessages([]);

    const echoTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topicName,
      messageType: topicType,
    });

    echoTopic.subscribe((msg) => {
      const payload = JSON.stringify(msg, null, 2);
      const row = {
        id: Date.now() + Math.random(),
        at: new Date().toLocaleTimeString(language === "ja" ? "ja-JP" : "en-US"),
        payload,
      };
      setTopicEchoMessages((prev) => [row, ...prev].slice(0, 30));
    });

    topicEchoSubRef.current = echoTopic;
    setTopicEchoRunning(true);
    setTopicEchoInfo(`${topicName} (${topicType}) を監視中`);
  };

  const serialTopicName = `serial_tx_${Math.max(0, Number.parseInt(serialTargetIdInput, 10) || 0)}`;

  const parseFloatSafe = (value, fallback = 0) => {
    const parsed = Number.parseFloat(value);
    return Number.isFinite(parsed) ? parsed : fallback;
  };

  const saveJsonExportToAppDirectory = async (category, data) => {
    try {
      const response = await fetch(`${backendBaseUrl}/api/json-exports/save`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ category, data }),
      });

      if (!response.ok) {
        const text = await response.text();
        throw new Error(`HTTP ${response.status}: ${text}`);
      }

      const result = await response.json();
      return {
        ok: true,
        fileName: result?.fileName || "",
      };
    } catch (error) {
      console.error("Failed to save JSON export to App.js directory:", error);
      return {
        ok: false,
        error,
      };
    }
  };

  const traceSampleMs = Math.max(50, Math.min(10000, Math.round(parseFloatSafe(traceSampleMsInput, 250))));
  const traceReplayMs = Math.max(50, Math.min(10000, Math.round(parseFloatSafe(traceReplayMsInput, 600))));

  const waypointDistanceThreshold = Math.max(0.01, parseFloatSafe(waypointDistanceThresholdInput, 0.2));
  const waypointYawThreshold = Math.max(0.5, parseFloatSafe(waypointYawThresholdInput, 15));

  const publishAutoDriveTarget = (x, y, yawRad) => {
    if (!autoDriveCmdRef.current) {
      return false;
    }
    autoDriveCmdRef.current.publish({
      data: [x, y, yawRad],
    });
    return true;
  };

  const applyWaypointToTarget = (index, shouldPublish = waypointAutoPublishEnabled) => {
    if (index < 0 || index >= waypoints.length) {
      return false;
    }
    const waypoint = waypoints[index];
    const yawRad = waypoint.yawDeg * Math.PI / 180;

    setTargetXInput(waypoint.x.toFixed(3));
    setTargetYInput(waypoint.y.toFixed(3));
    setTargetYawInput(waypoint.yawDeg.toFixed(1));

    if (shouldPublish) {
      if (!operationArmed) {
        setWaypointInfo(tr("操作ロック中のため目標送信はスキップしました", "Safety lock is ON, skipped publishing target"));
      } else if (!publishAutoDriveTarget(waypoint.x, waypoint.y, yawRad)) {
        setWaypointInfo(tr("ROS未接続のため目標送信できません", "Cannot publish target because ROS is not connected"));
      }
    }

    return true;
  };

  const applyTracePointToTarget = (index, shouldPublish = traceReplayAutoPublish) => {
    if (index < 0 || index >= tracePoints.length) {
      return false;
    }

    const point = tracePoints[index];
    const yawRad = point.yawDeg * Math.PI / 180;
    setTargetXInput(point.x.toFixed(3));
    setTargetYInput(point.y.toFixed(3));
    setTargetYawInput(point.yawDeg.toFixed(1));

    if (shouldPublish) {
      if (!operationArmed) {
        setTraceInfo(tr("操作ロック中のため目標送信はスキップしました", "Safety lock is ON, skipped publishing target"));
      } else if (!publishAutoDriveTarget(point.x, point.y, yawRad)) {
        setTraceInfo(tr("ROS未接続のため目標送信できません", "Cannot publish target because ROS is not connected"));
      }
    }

    return true;
  };

  const appendCurrentPoseToTrace = () => {
    const pose = tracePoseRef.current;
    const yawDeg = normalizeAngleDeg(pose.yaw * 180 / Math.PI);
    let addedLabel = "";
    let addedX = pose.x;
    let addedY = pose.y;

    setTracePoints((prev) => {
      const label = `T${prev.length + 1}`;
      addedLabel = label;
      const row = {
        id: `${Date.now()}-${Math.random().toString(16).slice(2)}`,
        x: pose.x,
        y: pose.y,
        yawDeg,
        label,
        at: new Date().toLocaleTimeString(traceLanguageRef.current === "ja" ? "ja-JP" : "en-US"),
      };
      addedX = row.x;
      addedY = row.y;
      return [...prev, row];
    });

    setTraceInfo(
      tr(
        `記録: ${addedLabel} (X ${addedX.toFixed(3)}, Y ${addedY.toFixed(3)})`,
        `Recorded: ${addedLabel} (X ${addedX.toFixed(3)}, Y ${addedY.toFixed(3)})`
      )
    );
  };

  const startTraceRecording = () => {
    setTraceRecording(true);
    setTraceInfo(tr(`記録開始 (${traceSampleMs} ms 間隔)`, `Recording started (${traceSampleMs} ms interval)`));
  };

  const stopTraceRecording = () => {
    setTraceRecording(false);
    setTraceInfo(tr("記録停止", "Recording stopped"));
  };

  const stopTraceReplay = () => {
    setTraceReplayRunning(false);
    setTraceReplayIndex(-1);
    setTraceInfo(tr("再生停止", "Replay stopped"));
  };

  const clearTracePoints = () => {
    setTracePoints([]);
    setTraceRecording(false);
    setTraceReplayRunning(false);
    setTraceReplayIndex(-1);
    setTraceInfo(tr("記録データを削除しました", "Trace data cleared"));
  };

  const removeTracePointById = (id) => {
    setTracePoints((prev) => {
      const removeIndex = prev.findIndex((point) => point.id === id);
      if (removeIndex < 0) {
        return prev;
      }
      const next = prev.filter((point) => point.id !== id);
      if (next.length === 0) {
        setTraceReplayRunning(false);
        setTraceReplayIndex(-1);
      } else if (traceReplayIndex >= next.length) {
        setTraceReplayIndex(next.length - 1);
      }
      return next;
    });
  };

  const startTraceReplay = () => {
    if (tracePoints.length === 0) {
      setTraceInfo(tr("再生する記録がありません", "No trace points to replay"));
      return;
    }
    setTraceReplayRunning(true);
    setTraceReplayIndex(0);
    applyTracePointToTarget(0, traceReplayAutoPublish);
    setTraceInfo(tr("再生開始", "Replay started"));
  };

  const exportTracePoints = async () => {
    if (tracePoints.length === 0) {
      setTraceInfo(tr("エクスポートする記録がありません", "No trace points to export"));
      return;
    }

    const exportData = {
      format: "nr26-trace-points",
      version: 1,
      exportedAt: new Date().toISOString(),
      points: tracePoints.map((point, index) => ({
        x: Number(point.x),
        y: Number(point.y),
        yawDeg: Number(point.yawDeg),
        label: point.label || `T${index + 1}`,
        at: point.at || "",
      })),
    };

    const json = JSON.stringify(exportData, null, 2);
    const blob = new Blob([json], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    const stamp = new Date().toISOString().replace(/[:.]/g, "-");
    link.href = url;
    link.download = `teaching_trace_${stamp}.json`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);

    const savedResult = await saveJsonExportToAppDirectory("teaching_trace", exportData);
    const saveSuffix = savedResult.ok
      ? tr(` / 保存: ${savedResult.fileName}`, ` / saved: ${savedResult.fileName}`)
      : tr(" / App.jsディレクトリ保存失敗", " / failed to save into App.js directory");

    setTraceInfo(
      tr(
        `記録をエクスポートしました (${tracePoints.length} 点)${saveSuffix}`,
        `Trace exported (${tracePoints.length} points)${saveSuffix}`
      )
    );
  };

  const normalizeImportedTracePoint = (rawPoint, index) => {
    const x = Number(rawPoint?.x);
    const y = Number(rawPoint?.y);

    let yawDeg = Number(rawPoint?.yawDeg);
    if (!Number.isFinite(yawDeg) && Number.isFinite(Number(rawPoint?.yawRad))) {
      yawDeg = Number(rawPoint.yawRad) * 180 / Math.PI;
    }
    if (!Number.isFinite(yawDeg) && Number.isFinite(Number(rawPoint?.yaw))) {
      yawDeg = Number(rawPoint.yaw) * 180 / Math.PI;
    }

    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(yawDeg)) {
      return null;
    }

    return {
      id: `${Date.now()}-${index}-${Math.random().toString(16).slice(2)}`,
      x,
      y,
      yawDeg: normalizeAngleDeg(yawDeg),
      label: typeof rawPoint?.label === "string" && rawPoint.label.trim() ? rawPoint.label.trim() : `T${index + 1}`,
      at: typeof rawPoint?.at === "string" ? rawPoint.at : "",
    };
  };

  const importTracePointsFromFile = async (event) => {
    const file = event.target.files?.[0];
    event.target.value = "";

    if (!file) {
      return;
    }

    try {
      const text = await file.text();
      const parsed = JSON.parse(text);
      const rawPoints = Array.isArray(parsed)
        ? parsed
        : Array.isArray(parsed?.points)
          ? parsed.points
          : null;

      if (!rawPoints) {
        setTraceInfo(tr("JSON形式が不正です (points配列が必要)", "Invalid JSON format (points array is required)"));
        return;
      }

      const normalizedPoints = rawPoints
        .map((point, index) => normalizeImportedTracePoint(point, index))
        .filter(Boolean);

      if (normalizedPoints.length === 0) {
        setTraceInfo(tr("有効な記録点が見つかりません", "No valid trace points found"));
        return;
      }

      setTraceRecording(false);
      setTraceReplayRunning(false);
      setTraceReplayIndex(-1);
      setTracePoints(normalizedPoints);
      setTraceInfo(
        tr(
          `記録をインポートしました (${normalizedPoints.length} 点)`,
          `Trace imported (${normalizedPoints.length} points)`
        )
      );
    } catch (error) {
      console.error("Failed to import trace points:", error);
      setTraceInfo(tr("インポートに失敗しました (JSONを確認してください)", "Import failed (check JSON file)"));
    }
  };

  const updateSerialElementCount = (nextCountRaw) => {
    const parsed = Number.parseInt(nextCountRaw, 10);
    const nextCount = Number.isFinite(parsed)
      ? Math.max(1, Math.min(64, parsed))
      : DEFAULT_PACKET_COUNT;

    setSerialElementCount(nextCount);
    setSerialValues((prev) => {
      const next = [...prev];
      if (next.length < nextCount) {
        while (next.length < nextCount) next.push(0);
      }
      if (next.length > nextCount) {
        next.length = nextCount;
      }
      return next;
    });
  };

  const updateSerialValue = (index, rawValue) => {
    const parsed = Number.parseInt(rawValue, 10);
    const safeValue = Number.isFinite(parsed)
      ? Math.max(-32768, Math.min(32767, parsed))
      : 0;

    setSerialValues((prev) => {
      const next = [...prev];
      next[index] = safeValue;
      return next;
    });
  };

  const renderSerialInputItem = (index) => {
    const label = PACKET_INDEX_LABELS[index] || `CH${index}`;
    const value = serialValues[index] ?? 0;
    return (
      <label className="serial-item" key={`${label}-${index}`}>
        <span className="serial-item-name">[{index}] {label}</span>
        <span className="serial-item-desc">{tr(describeActuatorJa(label), describeActuatorEn(label))}</span>
        <input
          className="connection-input"
          type="number"
          value={value}
          onChange={(e) => updateSerialValue(index, e.target.value)}
        />
      </label>
    );
  };

  const buildSerialPacketPayload = () => {
    const payload = serialValues.slice(0, serialElementCount);
    const sendPayload = [...payload];
    if (sendPayload.length < SERIAL_BRIDGE_MIN_ELEMENTS) {
      while (sendPayload.length < SERIAL_BRIDGE_MIN_ELEMENTS) {
        sendPayload.push(0);
      }
    }
    return {
      payload,
      sendPayload,
    };
  };

  const publishSerialPacket = (showStatus = true) => {
    if (!operationArmed) {
      if (showStatus) {
        setSerialPublishInfo("操作許可がOFFのため送信できません");
      }
      return false;
    }

    if (!rosRef.current) {
      if (showStatus) {
        setSerialPublishInfo("ROS未接続のため送信できません");
      }
      return false;
    }

    const targetId = Math.max(0, Number.parseInt(serialTargetIdInput, 10) || 0);
    const topicName = `serial_tx_${targetId}`;
    const { payload, sendPayload } = buildSerialPacketPayload();

    const serialTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topicName,
      messageType: "std_msgs/msg/Int16MultiArray",
    });

    serialTopic.publish({ data: sendPayload });
    if (showStatus) {
      setSerialPublishInfo(
        `${topicName} に ${sendPayload.length} 要素を送信 (入力 ${payload.length} 要素)`
      );
    }
    console.log("Serial TX publish:", topicName, sendPayload);
    return true;
  };

  const clearSerialPacket = () => {
    setSerialValues((prev) => prev.map(() => 0));
    setSerialPublishInfo("配列をクリアしました");
  };

  const applyAutoDriveFromCurrentPose = () => {
    setTargetXInput(poseX.toFixed(3));
    setTargetYInput(poseY.toFixed(3));
    setTargetYawInput((poseYaw * 180 / Math.PI).toFixed(1));
  };

  const publishOdomReset = () => {
    if (!operationArmed) {
      setAutoDriveCmdInfo("操作許可がOFFのためオドメトリリセット送信できません");
      return;
    }

    if (!odomResetCmdRef.current) {
      setAutoDriveCmdInfo("ROS未接続のためオドメトリリセット送信できません");
      return;
    }

    odomResetCmdRef.current.publish({
      data: true,
    });
    setAutoDriveCmdInfo("odom_reset にリセット要求を送信しました");
  };

  const saveTargetPose = () => {
    const x = parseFloatSafe(targetXInput);
    const y = parseFloatSafe(targetYInput);
    const yawDeg = parseFloatSafe(targetYawInput);
    const yawRad = yawDeg * Math.PI / 180;

    const newPose = {
      id: Date.now(),
      x: x,
      y: y,
      yaw: yawRad,
      yawDeg: yawDeg,
      timestamp: new Date().toLocaleTimeString(language === "ja" ? "ja-JP" : "en-US"),
      label: `目標${savedPosesList.length + 1}`,
    };

    setSavedPosesList([newPose, ...savedPosesList]);
    setAutoDriveCmdInfo(`目標値を保存しました: (${x.toFixed(3)}, ${y.toFixed(3)}, ${yawDeg.toFixed(1)}°)`);
  };

  const applySavedTargetPose = (pose) => {
    if (!autoDriveCmdRef.current) {
      setAutoDriveCmdInfo("ROS未接続のため送信できません");
      return;
    }

    setTargetXInput(pose.x.toFixed(3));
    setTargetYInput(pose.y.toFixed(3));
    setTargetYawInput(pose.yawDeg.toFixed(1));

    autoDriveCmdRef.current.publish({
      data: [pose.x, pose.y, pose.yaw],
    });

    setAutoDriveCmdInfo(`保存値 "${pose.label}" を送信しました`);
  };

  const deleteSavedPose = (id) => {
    setSavedPosesList(savedPosesList.filter((pose) => pose.id !== id));
  };

  const clearAllSavedPoses = () => {
    setSavedPosesList([]);
    setAutoDriveCmdInfo("すべての保存目標値をクリアしました");
  };

  const incrementTarget = (setter, currentValue, step) => {
    const current = parseFloatSafe(currentValue);
    const stepValue = parseFloatSafe(step);
    setter((current + stepValue).toFixed(1));
  };

  const decrementTarget = (setter, currentValue, step) => {
    const current = parseFloatSafe(currentValue);
    const stepValue = parseFloatSafe(step);
    setter((current - stepValue).toFixed(1));
  };

  const publishAutoDriveCommand = () => {
    if (!operationArmed) {
      setAutoDriveCmdInfo("操作許可がOFFのため送信できません");
      return;
    }

    if (!autoDriveCmdRef.current) {
      setAutoDriveCmdInfo("ROS未接続のため送信できません");
      return;
    }

    const tx = parseFloatSafe(targetXInput);
    const ty = parseFloatSafe(targetYInput);
    const tyawDeg = parseFloatSafe(targetYawInput);
    const tyawRad = tyawDeg * Math.PI / 180;

    autoDriveCmdRef.current.publish({
      data: [tx, ty, tyawRad],
    });

    setAutoDriveCmdInfo("r2_autodrive_cmd に目標座標を送信しました");
  };

  const publishArucoTargetCommand = () => {
    if (!operationArmed) {
      setArucoCmdInfo("操作許可がOFFのため送信できません");
      return;
    }

    if (!autoDriveCmdRef.current) {
      setArucoCmdInfo("ROS未接続のため送信できません");
      return;
    }

    const forward = parseFloatSafe(arucoTargetForwardInput);

    autoDriveCmdRef.current.publish({
      data: [forward, 0.0, 0.0],
    });

    setArucoCmdInfo("r2_autodrive_cmd に横位置目標を送信しました");
  };

  const publishArucoCameraOffset = () => {
    if (!operationArmed) {
      setArucoCmdInfo("操作許可がOFFのため送信できません");
      return;
    }

    if (!arucoCameraOffsetRef.current) {
      setArucoCmdInfo("ROS未接続のため送信できません");
      return;
    }

    const offsetX = parseFloatSafe(arucoCameraOffsetXInput);
    const offsetY = parseFloatSafe(arucoCameraOffsetYInput);

    arucoCameraOffsetRef.current.publish({
      data: [offsetX, offsetY],
    });

    setArucoCmdInfo("r2_aruco_camera_offset にカメラ位置を送信しました");
  };

  const publishArucoTargetId = () => {
    if (!operationArmed) {
      setArucoCmdInfo("操作許可がOFFのため送信できません");
      return;
    }

    if (!arucoTargetIdRef.current) {
      setArucoCmdInfo("ROS未接続のため送信できません");
      return;
    }

    const targetId = Math.trunc(parseFloatSafe(arucoTargetIdInput, -1));

    arucoTargetIdRef.current.publish({
      data: targetId,
    });

    setArucoCmdInfo("r2_aruco_target_id に追尾IDを送信しました");
  };

  const targetXValue = parseFloatSafe(targetXInput);
  const targetYValue = parseFloatSafe(targetYInput);
  const targetYawRadValue = parseFloatSafe(targetYawInput) * Math.PI / 180;
  const arucoTargetForwardCameraValue = parseFloatSafe(arucoTargetForwardInput);
  const arucoCameraOffsetXValue = parseFloatSafe(arucoCameraOffsetXInput, -0.1735);
  const arucoCameraOffsetYValue = parseFloatSafe(arucoCameraOffsetYInput, 0.0);
  const arucoTargetForwardRobotValue = arucoTargetForwardCameraValue + arucoCameraOffsetXValue;
  const arucoTargetWorldPose = driveMode === "ARUCO" && arucoDetectedWorldPose
    ? {
      x:
        arucoDetectedWorldPose.x -
        (Math.cos(poseYaw) * arucoTargetForwardRobotValue),
      y:
        arucoDetectedWorldPose.y -
        (Math.sin(poseYaw) * arucoTargetForwardRobotValue),
    }
    : null;

  const graphWidth = 360;
  const graphHeight = 250;
  const graphPadding = 28;
  const graphPoints = [
    { x: poseX, y: poseY },
    { x: targetXValue, y: targetYValue },
    ...waypoints.map((waypoint) => ({ x: waypoint.x, y: waypoint.y })),
    ...(arucoDetectedWorldPose ? [{ x: arucoDetectedWorldPose.x, y: arucoDetectedWorldPose.y }] : []),
    ...(arucoTargetWorldPose ? [{ x: arucoTargetWorldPose.x, y: arucoTargetWorldPose.y }] : []),
  ];

  const graphMinXRaw = Math.min(...graphPoints.map((p) => p.x));
  const graphMaxXRaw = Math.max(...graphPoints.map((p) => p.x));
  const graphMinYRaw = Math.min(...graphPoints.map((p) => p.y));
  const graphMaxYRaw = Math.max(...graphPoints.map((p) => p.y));
  const graphSpan = Math.max(graphMaxXRaw - graphMinXRaw, graphMaxYRaw - graphMinYRaw, 1);
  const graphMargin = Math.max(graphSpan * 0.2, 0.3);

  const graphMinX = graphMinXRaw - graphMargin;
  const graphMaxX = graphMaxXRaw + graphMargin;
  const graphMinY = graphMinYRaw - graphMargin;
  const graphMaxY = graphMaxYRaw + graphMargin;

  const graphInnerWidth = graphWidth - graphPadding * 2;
  const graphInnerHeight = graphHeight - graphPadding * 2;

  const toGraphX = (x) =>
    graphPadding + ((x - graphMinX) / Math.max(graphMaxX - graphMinX, 1e-6)) * graphInnerWidth;
  const toGraphY = (y) =>
    graphHeight - graphPadding - ((y - graphMinY) / Math.max(graphMaxY - graphMinY, 1e-6)) * graphInnerHeight;

  const mapClientPointToGraph = (svg, clientX, clientY) => {
    if (!svg) {
      return null;
    }

    const rect = svg.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) {
      return null;
    }

    const normalizedX = (clientX - rect.left) / rect.width;
    const normalizedY = (clientY - rect.top) / rect.height;
    const viewX = Math.max(graphPadding, Math.min(graphWidth - graphPadding, normalizedX * graphWidth));
    const viewY = Math.max(graphPadding, Math.min(graphHeight - graphPadding, normalizedY * graphHeight));

    const x = graphMinX + ((viewX - graphPadding) / Math.max(graphInnerWidth, 1e-6)) * (graphMaxX - graphMinX);
    const y = graphMinY + ((graphHeight - graphPadding - viewY) / Math.max(graphInnerHeight, 1e-6)) * (graphMaxY - graphMinY);

    return { x, y };
  };

  const addWaypointFromGraphClientPoint = (clientX, clientY) => {
    const graphPoint = mapClientPointToGraph(waypointGraphRef.current, clientX, clientY);
    if (!graphPoint) {
      return;
    }

    const yawDeg = parseFloatSafe(targetYawInput);
    let appendedLabel = "";
    let appendedX = graphPoint.x;
    let appendedY = graphPoint.y;

    setWaypoints((prev) => {
      const nextLabel = `WP${prev.length + 1}`;
      appendedLabel = nextLabel;
      const newWaypoint = {
        id: `${Date.now()}-${Math.random().toString(16).slice(2)}`,
        x: graphPoint.x,
        y: graphPoint.y,
        yawDeg: yawDeg,
        label: nextLabel,
      };
      appendedX = newWaypoint.x;
      appendedY = newWaypoint.y;
      return [...prev, newWaypoint];
    });

    if (activeWaypointIndex < 0) {
      setActiveWaypointIndex(0);
    }
    setWaypointInfo(
      tr(
        `ウェイポイントを追加: ${appendedLabel} (X ${appendedX.toFixed(3)}, Y ${appendedY.toFixed(3)})`,
        `Waypoint added: ${appendedLabel} (X ${appendedX.toFixed(3)}, Y ${appendedY.toFixed(3)})`
      )
    );
  };

  const updateTargetFromGraphClientPoint = (clientX, clientY) => {
    const graphPoint = mapClientPointToGraph(poseGraphRef.current, clientX, clientY);
    if (!graphPoint) {
      return;
    }

    setTargetXInput(graphPoint.x.toFixed(3));
    setTargetYInput(graphPoint.y.toFixed(3));
    setAutoDriveCmdInfo(
      tr(
        `グラフで目標値を設定: X ${graphPoint.x.toFixed(3)}, Y ${graphPoint.y.toFixed(3)}`,
        `Target set from graph: X ${graphPoint.x.toFixed(3)}, Y ${graphPoint.y.toFixed(3)}`
      )
    );
  };

  const startWaypointNavigation = () => {
    if (waypoints.length === 0) {
      setWaypointInfo(tr("ウェイポイントが未登録です", "No waypoints are registered"));
      return;
    }

    const startIndex = activeWaypointIndex >= 0 && activeWaypointIndex < waypoints.length
      ? activeWaypointIndex
      : 0;
    setActiveWaypointIndex(startIndex);
    setWaypointAutoAdvanceEnabled(true);
    applyWaypointToTarget(startIndex, waypointAutoPublishEnabled);
    setWaypointInfo(
      tr(
        `追従開始: ${waypoints[startIndex].label}`,
        `Waypoint navigation started: ${waypoints[startIndex].label}`
      )
    );
  };

  const stopWaypointNavigation = () => {
    setWaypointAutoAdvanceEnabled(false);
    setWaypointInfo(tr("ウェイポイント追従を停止しました", "Waypoint navigation stopped"));
  };

  const clearWaypoints = () => {
    setWaypoints([]);
    setActiveWaypointIndex(-1);
    setWaypointAutoAdvanceEnabled(false);
    setWaypointInfo(tr("ウェイポイントをクリアしました", "Waypoints cleared"));
  };

  const removeWaypointById = (id) => {
    setWaypoints((prev) => {
      const removeIndex = prev.findIndex((waypoint) => waypoint.id === id);
      if (removeIndex < 0) {
        return prev;
      }

      const next = prev.filter((waypoint) => waypoint.id !== id);
      setActiveWaypointIndex((current) => {
        if (next.length === 0) {
          setWaypointAutoAdvanceEnabled(false);
          return -1;
        }
        if (current === removeIndex) {
          return Math.min(removeIndex, next.length - 1);
        }
        if (current > removeIndex) {
          return current - 1;
        }
        return current;
      });
      return next;
    });
  };

  const setWaypointAsActive = (index, publishNow = false) => {
    if (index < 0 || index >= waypoints.length) {
      return;
    }
    setActiveWaypointIndex(index);
    applyWaypointToTarget(index, publishNow);
    setWaypointInfo(
      tr(
        `アクティブウェイポイント: ${waypoints[index].label}`,
        `Active waypoint: ${waypoints[index].label}`
      )
    );
  };

  const handlePoseGraphPointerDown = (event) => {
    if (event.pointerType === "mouse" && event.button !== 0) {
      return;
    }
    poseGraphPointerActiveRef.current = true;
    event.currentTarget.setPointerCapture?.(event.pointerId);
    updateTargetFromGraphClientPoint(event.clientX, event.clientY);
  };

  const handlePoseGraphPointerMove = (event) => {
    if (!poseGraphPointerActiveRef.current) {
      return;
    }
    updateTargetFromGraphClientPoint(event.clientX, event.clientY);
  };

  const releasePoseGraphPointer = () => {
    poseGraphPointerActiveRef.current = false;
  };

  const handleWaypointGraphPointerDown = (event) => {
    if (event.pointerType === "mouse" && event.button !== 0) {
      return;
    }
    event.currentTarget.setPointerCapture?.(event.pointerId);
    addWaypointFromGraphClientPoint(event.clientX, event.clientY);
  };

  const waypointPolylinePoints = waypoints
    .map((waypoint) => `${toGraphX(waypoint.x)},${toGraphY(waypoint.y)}`)
    .join(" ");

  const yawOffsetRad = (parseFloatSafe(yawOffsetDegInput, 0) * Math.PI) / 180;

  const currentX = toGraphX(poseX);
  const currentY = toGraphY(poseY);
  const targetX = toGraphX(targetXValue);
  const targetY = toGraphY(targetYValue);
  const arucoDetectedX = arucoDetectedWorldPose ? toGraphX(arucoDetectedWorldPose.x) : null;
  const arucoDetectedY = arucoDetectedWorldPose ? toGraphY(arucoDetectedWorldPose.y) : null;
  const arucoTargetX = arucoTargetWorldPose ? toGraphX(arucoTargetWorldPose.x) : null;
  const arucoTargetY = arucoTargetWorldPose ? toGraphY(arucoTargetWorldPose.y) : null;
  const arrowLength = 20;

  const currentYaw = normalizeYawRad(poseYaw + yawOffsetRad);
  const targetYaw = normalizeYawRad(targetYawRadValue + yawOffsetRad);

  const currentArrowX = currentX + Math.cos(currentYaw) * arrowLength;
  const currentArrowY = currentY - Math.sin(currentYaw) * arrowLength;
  const targetArrowX = targetX + Math.cos(targetYaw) * arrowLength;
  const targetArrowY = targetY - Math.sin(targetYaw) * arrowLength;
  const wallAngleDeg = normalizeAngleDeg((wallAngleRad * 180) / Math.PI);
  const wallGaugeSize = 240;
  const wallGaugeCx = wallGaugeSize / 2;
  const wallGaugeCy = wallGaugeSize / 2;
  const wallGaugeRadius = 84;
  const wallGaugeNeedleLen = 74;
  const wallGaugeNeedleX = wallGaugeCx + Math.cos(wallAngleRad) * wallGaugeNeedleLen;
  const wallGaugeNeedleY = wallGaugeCy - Math.sin(wallAngleRad) * wallGaugeNeedleLen;

  // Trace points graph calculation
  const traceGraphPoints = [
    { x: poseX, y: poseY },
    ...tracePoints.map((point) => ({ x: point.x, y: point.y })),
  ];

  const traceGraphMinXRaw = Math.min(...traceGraphPoints.map((p) => p.x));
  const traceGraphMaxXRaw = Math.max(...traceGraphPoints.map((p) => p.x));
  const traceGraphMinYRaw = Math.min(...traceGraphPoints.map((p) => p.y));
  const traceGraphMaxYRaw = Math.max(...traceGraphPoints.map((p) => p.y));
  const traceGraphSpan = Math.max(traceGraphMaxXRaw - traceGraphMinXRaw, traceGraphMaxYRaw - traceGraphMinYRaw, 1);
  const traceGraphMargin = Math.max(traceGraphSpan * 0.2, 0.3);

  const traceGraphMinX = traceGraphMinXRaw - traceGraphMargin;
  const traceGraphMaxX = traceGraphMaxXRaw + traceGraphMargin;
  const traceGraphMinY = traceGraphMinYRaw - traceGraphMargin;
  const traceGraphMaxY = traceGraphMaxYRaw + traceGraphMargin;

  const toTraceGraphX = (x) =>
    graphPadding + ((x - traceGraphMinX) / Math.max(traceGraphMaxX - traceGraphMinX, 1e-6)) * graphInnerWidth;
  const toTraceGraphY = (y) =>
    graphHeight - graphPadding - ((y - traceGraphMinY) / Math.max(traceGraphMaxY - traceGraphMinY, 1e-6)) * graphInnerHeight;

  const traceAxisXVisible = traceGraphMinX <= 0 && traceGraphMaxX >= 0;
  const traceAxisYVisible = traceGraphMinY <= 0 && traceGraphMaxY >= 0;
  const traceGridStep = chooseGridStep(Math.max(traceGraphMaxX - traceGraphMinX, traceGraphMaxY - traceGraphMinY));

  const traceGridXValues = [];
  const traceStartGridX = Math.ceil(traceGraphMinX / traceGridStep) * traceGridStep;
  for (let value = traceStartGridX; value <= traceGraphMaxX + 1e-9; value += traceGridStep) {
    traceGridXValues.push(Number(value.toFixed(6)));
  }

  const traceGridYValues = [];
  const traceStartGridY = Math.ceil(traceGraphMinY / traceGridStep) * traceGridStep;
  for (let value = traceStartGridY; value <= traceGraphMaxY + 1e-9; value += traceGridStep) {
    traceGridYValues.push(Number(value.toFixed(6)));
  }

  const axisXVisible = graphMinX <= 0 && graphMaxX >= 0;
  const axisYVisible = graphMinY <= 0 && graphMaxY >= 0;
  const gridStep = chooseGridStep(Math.max(graphMaxX - graphMinX, graphMaxY - graphMinY));

  const gridXValues = [];
  const startGridX = Math.ceil(graphMinX / gridStep) * gridStep;
  for (let value = startGridX; value <= graphMaxX + 1e-9; value += gridStep) {
    gridXValues.push(Number(value.toFixed(6)));
  }

  const gridYValues = [];
  const startGridY = Math.ceil(graphMinY / gridStep) * gridStep;
  for (let value = startGridY; value <= graphMaxY + 1e-9; value += gridStep) {
    gridYValues.push(Number(value.toFixed(6)));
  }

  // Wall surface estimation graph calculation (Cartesian coordinate system)
  const wallGraphWidth = 300;
  const wallGraphHeight = 300;
  const wallGraphPadding = 28;

  // Rotate raw points 90 degrees counterclockwise for display: (x, y) -> (-y, x)
  const wallDisplayPoints = wallFilteredPoints.map((p) => ({ x: -p.y, y: p.x }));

  let wallGraphMinXRaw = 0, wallGraphMaxXRaw = 0;
  let wallGraphMinYRaw = 0, wallGraphMaxYRaw = 0;

  if (wallDisplayPoints.length > 0) {
    wallGraphMinXRaw = Math.min(...wallDisplayPoints.map((p) => p.x));
    wallGraphMaxXRaw = Math.max(...wallDisplayPoints.map((p) => p.x));
    wallGraphMinYRaw = Math.min(...wallDisplayPoints.map((p) => p.y));
    wallGraphMaxYRaw = Math.max(...wallDisplayPoints.map((p) => p.y));
  } else {
    wallGraphMinXRaw = wallGraphMaxXRaw = 0;
    wallGraphMinYRaw = wallGraphMaxYRaw = 0;
  }

  const wallGraphExtentRaw = Math.max(
    Math.abs(wallGraphMinXRaw),
    Math.abs(wallGraphMaxXRaw),
    Math.abs(wallGraphMinYRaw),
    Math.abs(wallGraphMaxYRaw),
    1
  );
  const wallGraphMargin = Math.max(wallGraphExtentRaw * 0.2, 0.5);
  const wallGraphExtent = wallGraphExtentRaw + wallGraphMargin;

  const wallGraphMinX = -wallGraphExtent;
  const wallGraphMaxX = wallGraphExtent;
  const wallGraphMinY = -wallGraphExtent;
  const wallGraphMaxY = wallGraphExtent;

  const wallGraphInnerWidth = wallGraphWidth - wallGraphPadding * 2;
  const wallGraphInnerHeight = wallGraphHeight - wallGraphPadding * 2;

  const toWallGraphX = (x) =>
    wallGraphPadding + ((x - wallGraphMinX) / Math.max(wallGraphMaxX - wallGraphMinX, 1e-6)) * wallGraphInnerWidth;
  const toWallGraphY = (y) =>
    wallGraphHeight - wallGraphPadding - ((y - wallGraphMinY) / Math.max(wallGraphMaxY - wallGraphMinY, 1e-6)) * wallGraphInnerHeight;

  const wallAxisXVisible = wallGraphMinX <= 0 && wallGraphMaxX >= 0;
  const wallAxisYVisible = wallGraphMinY <= 0 && wallGraphMaxY >= 0;
  const wallGridStep = chooseGridStep(Math.max(wallGraphMaxX - wallGraphMinX, wallGraphMaxY - wallGraphMinY));

  const wallGridXValues = [];
  const wallStartGridX = Math.ceil(wallGraphMinX / wallGridStep) * wallGridStep;
  for (let value = wallStartGridX; value <= wallGraphMaxX + 1e-9; value += wallGridStep) {
    wallGridXValues.push(Number(value.toFixed(6)));
  }

  const wallGridYValues = [];
  const wallStartGridY = Math.ceil(wallGraphMinY / wallGridStep) * wallGridStep;
  for (let value = wallStartGridY; value <= wallGraphMaxY + 1e-9; value += wallGridStep) {
    wallGridYValues.push(Number(value.toFixed(6)));
  }

  // Debug logging
  if (isPageActive("wall-angle")) {
    const debugInfo = {
      wallFilteredPointsCount: wallFilteredPoints.length,
      wallGridXCount: wallGridXValues.length,
      wallGridYCount: wallGridYValues.length,
      wallGraphMinX,
      wallGraphMaxX,
      wallGraphMinY,
      wallGraphMaxY,
      wallGridStep,
    };
    console.log("Wall surface estimation graph debug:", debugInfo);
  }

  // RANSAC line endpoints calculation in rotated display coordinates
  let ransacLinePoint1 = null, ransacLinePoint2 = null;
  if (wallRansacParams) {
    const lineA = -wallRansacParams.b;
    const lineB = wallRansacParams.a;
    const lineC = wallRansacParams.c;

    if (Math.abs(lineB) > 1e-6) {
      const y1 = -(lineA * wallGraphMinX + lineC) / lineB;
      const y2 = -(lineA * wallGraphMaxX + lineC) / lineB;
      ransacLinePoint1 = { x: wallGraphMinX, y: y1 };
      ransacLinePoint2 = { x: wallGraphMaxX, y: y2 };
    } else if (Math.abs(lineA) > 1e-6) {
      const x1 = -(lineB * wallGraphMinY + lineC) / lineA;
      const x2 = -(lineB * wallGraphMaxY + lineC) / lineA;
      ransacLinePoint1 = { x: x1, y: wallGraphMinY };
      ransacLinePoint2 = { x: x2, y: wallGraphMaxY };
    }
  }

  useEffect(() => {
    tracePoseRef.current = { x: poseX, y: poseY, yaw: poseYaw };
  }, [poseX, poseY, poseYaw]);

  useEffect(() => {
    traceLanguageRef.current = language;
  }, [language]);

  useEffect(() => {
    if (!operationArmed && serialPeriodicEnabled) {
      setSerialPeriodicEnabled(false);
      setSerialPublishInfo("操作許可がOFFのため定期送信を停止しました");
    }
  }, [operationArmed, serialPeriodicEnabled]);

  useEffect(() => {
    if (!waypointAutoAdvanceEnabled) {
      return;
    }

    if (activeWaypointIndex < 0 || activeWaypointIndex >= waypoints.length) {
      return;
    }

    const activeWaypoint = waypoints[activeWaypointIndex];
    const distanceError = Math.hypot(activeWaypoint.x - poseX, activeWaypoint.y - poseY);
    const yawErrorDeg = Math.abs(normalizeAngleDeg(activeWaypoint.yawDeg - (poseYaw * 180 / Math.PI)));

    const distanceReached = distanceError <= waypointDistanceThreshold;
    const yawReached = !waypointCheckYaw || yawErrorDeg <= waypointYawThreshold;
    if (!distanceReached || !yawReached) {
      return;
    }

    if (activeWaypointIndex + 1 >= waypoints.length) {
      setWaypointAutoAdvanceEnabled(false);
      setWaypointInfo(
        tr(
          `最終ウェイポイント ${activeWaypoint.label} に到達しました`,
          `Reached final waypoint ${activeWaypoint.label}`
        )
      );
      return;
    }

    const nextIndex = activeWaypointIndex + 1;
    setActiveWaypointIndex(nextIndex);
    applyWaypointToTarget(nextIndex, waypointAutoPublishEnabled);
    setWaypointInfo(
      tr(
        `${activeWaypoint.label} に到達。次の ${waypoints[nextIndex].label} へ切替`,
        `${activeWaypoint.label} reached. Switched to ${waypoints[nextIndex].label}`
      )
    );
  }, [
    waypointAutoAdvanceEnabled,
    activeWaypointIndex,
    waypoints,
    poseX,
    poseY,
    poseYaw,
    waypointDistanceThreshold,
    waypointCheckYaw,
    waypointYawThreshold,
    waypointAutoPublishEnabled,
  ]);

  useEffect(() => {
    if (traceRecordTimerRef.current) {
      clearInterval(traceRecordTimerRef.current);
      traceRecordTimerRef.current = null;
    }

    if (!traceRecording) {
      return;
    }

    appendCurrentPoseToTrace();
    traceRecordTimerRef.current = setInterval(() => {
      appendCurrentPoseToTrace();
    }, traceSampleMs);

    return () => {
      if (traceRecordTimerRef.current) {
        clearInterval(traceRecordTimerRef.current);
        traceRecordTimerRef.current = null;
      }
    };
  }, [traceRecording, traceSampleMs]);

  useEffect(() => {
    if (traceReplayTimerRef.current) {
      clearInterval(traceReplayTimerRef.current);
      traceReplayTimerRef.current = null;
    }

    if (!traceReplayRunning || tracePoints.length === 0) {
      return;
    }

    traceReplayTimerRef.current = setInterval(() => {
      setTraceReplayIndex((prev) => {
        const currentIndex = prev < 0 ? 0 : prev;
        const nextIndex = currentIndex + 1;

        if (nextIndex >= tracePoints.length) {
          if (traceReplayLoop) {
            applyTracePointToTarget(0, traceReplayAutoPublish);
            setTraceInfo(tr("再生ループ: 先頭に戻りました", "Replay loop: back to start"));
            return 0;
          }

          setTraceReplayRunning(false);
          setTraceInfo(tr("再生完了", "Replay completed"));
          return tracePoints.length - 1;
        }

        applyTracePointToTarget(nextIndex, traceReplayAutoPublish);
        return nextIndex;
      });
    }, traceReplayMs);

    return () => {
      if (traceReplayTimerRef.current) {
        clearInterval(traceReplayTimerRef.current);
        traceReplayTimerRef.current = null;
      }
    };
  }, [traceReplayRunning, traceReplayMs, tracePoints, traceReplayLoop, traceReplayAutoPublish]);

  useEffect(() => {
    if (serialPeriodicTimerRef.current) {
      clearInterval(serialPeriodicTimerRef.current);
      serialPeriodicTimerRef.current = null;
    }

    if (!serialPeriodicEnabled || !operationArmed) {
      return;
    }

    const parsedHz = Number.parseFloat(serialPeriodicHz);
    const safeHz = Number.isFinite(parsedHz)
      ? Math.max(0.5, Math.min(100, parsedHz))
      : 10;
    const intervalMs = Math.max(10, Math.round(1000 / safeHz));

    setSerialPublishInfo(`定期送信中: ${safeHz} Hz`);
    serialPeriodicTimerRef.current = setInterval(() => {
      publishSerialPacket(false);
    }, intervalMs);

    return () => {
      if (serialPeriodicTimerRef.current) {
        clearInterval(serialPeriodicTimerRef.current);
        serialPeriodicTimerRef.current = null;
      }
    };
  }, [serialPeriodicEnabled, serialPeriodicHz, serialTargetIdInput, serialElementCount, serialValues, operationArmed]);

  const updateCommand = (value) => {
    commandValueRef.current = value;
    setCommandValue(value);
  };

  const resetAllControls = () => {
    updateCommand(0);

    const nextButtons = Array(14).fill(0);
    const nextAxes = Array(8).fill(0);

    buttonsRef.current = nextButtons;
    axesRef.current = nextAxes;
    setButtons(nextButtons);
    setAxes(nextAxes);
  };

  const getHoldHandlers = (onPress, onRelease) => ({
    onPointerDown: (event) => {
      console.log("Pointer DOWN:", event.pointerId);
      event.preventDefault();
      event.stopPropagation();
      if (event.currentTarget.setPointerCapture) {
        event.currentTarget.setPointerCapture(event.pointerId);
      }
      onPress();
    },
    onPointerUp: (event) => {
      console.log("Pointer UP:", event.pointerId);
      event.preventDefault();
      event.stopPropagation();
      onRelease();
    },
    onPointerCancel: (event) => {
      console.log("Pointer CANCEL:", event.pointerId);
      event.preventDefault();
      event.stopPropagation();
      onRelease();
    },
    onLostPointerCapture: (event) => {
      console.log("Lost Pointer Capture:", event.pointerId);
      event.preventDefault();
      onRelease();
    },
    onContextMenu: (event) => event.preventDefault(),
  });

  const setPsButton = (index, isPressed) => {
    const nextButtons = [...buttonsRef.current];
    nextButtons[index] = isPressed ? 1 : 0;
    buttonsRef.current = nextButtons;
    setButtons(nextButtons);
  };

  const setPsAxis = (index, value) => {
    const nextAxes = [...axesRef.current];
    nextAxes[index] = value;
    axesRef.current = nextAxes;
    setAxes(nextAxes);
  };

  const getButtonPressProps = (index) =>
    getHoldHandlers(
      () => setPsButton(index, true),
      () => setPsButton(index, false)
    );

  const getAxisPressProps = (index, value) =>
    getHoldHandlers(
      () => setPsAxis(index, value),
      () => setPsAxis(index, 0)
    );

  useEffect(() => {
    const resetOnBlur = () => {
      resetAllControls();
    };

    const resetOnHidden = () => {
      if (document.visibilityState === "hidden") {
        resetAllControls();
      }
    };

    window.addEventListener("blur", resetOnBlur);
    window.addEventListener("visibilitychange", resetOnHidden);

    return () => {
      window.removeEventListener("blur", resetOnBlur);
      window.removeEventListener("visibilitychange", resetOnHidden);
    };
  }, []);

  useEffect(() => {
    virtualOdomEnabledRef.current = virtualOdomEnabled;
  }, [virtualOdomEnabled]);

  useEffect(() => {
    arucoTargetIdValueRef.current = Math.trunc(parseFloatSafe(arucoTargetIdInput, -1));
  }, [arucoTargetIdInput]);

  useEffect(() => {
    arucoCameraOffsetXRef.current = parseFloatSafe(arucoCameraOffsetXInput, -0.1735);
  }, [arucoCameraOffsetXInput]);

  useEffect(() => {
    arucoCameraOffsetYRef.current = parseFloatSafe(arucoCameraOffsetYInput, 0.0);
  }, [arucoCameraOffsetYInput]);

  useEffect(() => {
    setStatus("接続中...");

    // ROS接続
    rosRef.current = new ROSLIB.Ros({
      url: rosUrl,
    });

    rosRef.current.on("connection", () => {
      setStatus("接続OK");
      console.log("Connected to ROS:", rosUrl);
      refreshTopicList();
      refreshSerialBridgeStatus();
      refreshSerialBridgeLogs();
    });

    rosRef.current.on("error", (error) => {
      setStatus("エラー");
      console.log("Error:", error);
    });

    rosRef.current.on("close", () => {
      setStatus("切断");
      console.log("Connection closed");
    });

    // 古いjoyトピックをクリーンアップ
    if (joyRef.current) {
      try {
        joyRef.current.unsubscribe?.();
        console.log("Old joy topic unsubscribed");
      } catch (error) {
        console.warn("Error unsubscribing old joy topic:", error);
      }
    }

    // Topic定義
    commandRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/command",
      messageType: "std_msgs/msg/Int32MultiArray",
    });

    joyRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: joyTopicName,
      messageType: "sensor_msgs/msg/Joy",
    });
    console.log("Joy topic created:", joyTopicName);

    virtualOdomPubRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: virtualOdomTopicName,
      messageType: "std_msgs/msg/Float32MultiArray",
    });
    console.log("Virtual odometry topic created:", virtualOdomTopicName);

    odomRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "odom_xy_yaw",
      messageType: "std_msgs/msg/Float32MultiArray",
    });
    odomRef.current.subscribe((msg) => {
      if (virtualOdomEnabledRef.current) return;
      if (!msg?.data || msg.data.length < 3) return;
      setPoseX(Number(msg.data[0]) || 0);
      setPoseY(Number(msg.data[1]) || 0);
      setPoseYaw(Number(msg.data[2]) || 0);
    });

    driveModeRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2_drive_mode",
      messageType: "std_msgs/msg/String",
    });
    driveModeRef.current.subscribe((msg) => {
      const nextMode = (msg?.data || "").toUpperCase();
      if (nextMode === "AUTO" || nextMode === "MANUAL" || nextMode === "ARUCO") {
        setDriveMode(nextMode);
      }
    });

    autoDriveCmdRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2_autodrive_cmd",
      messageType: "std_msgs/msg/Float32MultiArray",
    });

    arucoCameraOffsetRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2_aruco_camera_offset",
      messageType: "std_msgs/msg/Float32MultiArray",
    });

    arucoTargetIdRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2_aruco_target_id",
      messageType: "std_msgs/msg/Int32",
    });

    arucoIdSubRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/aruco_id",
      messageType: "std_msgs/msg/Int32",
    });
    arucoIdSubRef.current.subscribe((msg) => {
      const nextId = Number(msg?.data);
      const normalizedId = Number.isFinite(nextId) ? nextId : null;
      arucoDetectedIdRef.current = normalizedId;
      setArucoDetectedId(normalizedId);
    });

    arucoPoseSubRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/aruco_pose",
      messageType: "geometry_msgs/msg/PoseStamped",
    });
    arucoPoseSubRef.current.subscribe((msg) => {
      const markerX = Number(msg?.pose?.position?.x);
      const markerZ = Number(msg?.pose?.position?.z);
      if (!Number.isFinite(markerX) || !Number.isFinite(markerZ)) {
        return;
      }

      const targetId = arucoTargetIdValueRef.current;
      const currentId = arucoDetectedIdRef.current;
      if (targetId >= 0 && currentId !== targetId) {
        return;
      }

      // Camera faces robot-left: camera x->robot forward, camera z->robot left
      const measuredForward = markerX + arucoCameraOffsetXRef.current;
      const measuredLateral = markerZ + arucoCameraOffsetYRef.current;

      const pose = tracePoseRef.current;
      const robotRight = -measuredLateral;
      const robotForward = measuredForward;
      const dxWorld = robotRight * Math.cos(pose.yaw) - robotForward * Math.sin(pose.yaw);
      const dyWorld = robotRight * Math.sin(pose.yaw) + robotForward * Math.cos(pose.yaw);

      setArucoDetectedWorldPose({
        x: pose.x + dxWorld,
        y: pose.y + dyWorld,
      });
    });

    wallAngleSubRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: wallAngleTopicName,
      messageType: "std_msgs/msg/Float64",
    });
    wallAngleSubRef.current.subscribe((msg) => {
      const nextAngle = Number(msg?.data);
      if (!Number.isFinite(nextAngle)) {
        return;
      }
      setWallAngleRad(nextAngle);
      setWallAngleUpdatedAt(new Date().toLocaleTimeString());
    });

    // 点群を購読 (PointCloud2)
    const filteredPointsTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/wall_detection/filtered_points",
      messageType: "sensor_msgs/msg/PointCloud2",
    });
    filteredPointsTopic.subscribe((msg) => {
      try {
        const points = [];
        if (msg?.width > 0 && msg?.data) {
          // Handle multiple PointCloud2 data encodings from rosbridge
          let dataArray = null;
          if (msg.data instanceof Uint8Array) {
            dataArray = msg.data;
          } else if (Array.isArray(msg.data)) {
            dataArray = new Uint8Array(msg.data);
          } else if (typeof msg.data === "string") {
            const binary = atob(msg.data);
            dataArray = new Uint8Array(binary.length);
            for (let i = 0; i < binary.length; i++) {
              dataArray[i] = binary.charCodeAt(i);
            }
          }

          if (!dataArray || dataArray.byteLength === 0) {
            setWallFilteredPoints([]);
            return;
          }

          const dataView = new DataView(dataArray.buffer, dataArray.byteOffset, dataArray.byteLength);

          // Find field offsets
          const xField = msg.fields?.find(f => f.name === "x");
          const yField = msg.fields?.find(f => f.name === "y");
          const xOffset = xField?.offset || 0;
          const yOffset = yField?.offset || 4;
          const pointStep = Number(msg.point_step) || 8;
          const pointCount = Math.min(Number(msg.width) || 0, Math.floor(dataArray.byteLength / pointStep));
          const littleEndian = !msg.is_bigendian;

          for (let i = 0; i < pointCount; i++) {
            const byteOffset = i * pointStep;
            try {
              const x = dataView.getFloat32(byteOffset + xOffset, littleEndian);
              const y = dataView.getFloat32(byteOffset + yOffset, littleEndian);
              if (Number.isFinite(x) && Number.isFinite(y)) {
                points.push({ x, y });
              }
            } catch (e) {
              // Skip invalid points
            }
          }
        }
        setWallFilteredPoints(points);
      } catch (e) {
        console.error("Error parsing filtered points:", e);
      }
    });

    // RANSACパラメータを購読 (Float64MultiArray)
    const ransacParamsTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/wall_detection/ransac_params",
      messageType: "std_msgs/msg/Float64MultiArray",
    });
    ransacParamsTopic.subscribe((msg) => {
      try {
        if (msg?.data && msg.data.length >= 5) {
          setWallRansacParams({
            a: msg.data[0],
            b: msg.data[1],
            c: msg.data[2],
            inliers: Math.round(msg.data[3]),
            total: Math.round(msg.data[4]),
          });
        }
      } catch (e) {
        console.error("Error parsing RANSAC params:", e);
      }
    });

    const cubeDetectedTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/cube_detection/detected",
      messageType: "std_msgs/msg/Bool",
    });
    cubeDetectedTopic.subscribe((msg) => {
      setCubeDetected(Boolean(msg?.data));
      setCubeUpdatedAt(new Date().toLocaleTimeString());
    });

    const cubeInfoTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/cube_detection/info",
      messageType: "std_msgs/msg/Float32MultiArray",
    });
    cubeInfoTopic.subscribe((msg) => {
      const data = Array.isArray(msg?.data) ? msg.data : [];
      if (data.length < 8) {
        return;
      }

      setCubeDetected(Boolean(data[0]));
      setCubeInfo({
        cxNorm: Number(data[1]) || 0,
        cyNorm: Number(data[2]) || 0,
        widthNorm: Number(data[3]) || 0,
        heightNorm: Number(data[4]) || 0,
        depthM: Number(data[5]) || 0,
        score: Number(data[6]) || 0,
        areaPx: Math.round(Number(data[7]) || 0),
      });
      setCubeUpdatedAt(new Date().toLocaleTimeString());
    });

    odomResetCmdRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "odom_reset",
      messageType: "std_msgs/msg/Bool",
    });

    taskStateCommandRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2/task_state",
      messageType: "std_msgs/msg/Int32",
    });

    taskColorCommandRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2/task_color",
      messageType: "std_msgs/msg/Int32",
    });

    taskCellCommandRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2/task_cell",
      messageType: "std_msgs/msg/Int32",
    });

    taskTransitionModeRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2/task_transition_mode",
      messageType: "std_msgs/msg/Int32",
    });

    taskStateSequenceRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2/task_state_sequence",
      messageType: "std_msgs/msg/Int32MultiArray",
    });

    taskStatePoseRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2/task_state_pose",
      messageType: "std_msgs/msg/Float32MultiArray",
    });

    taskStateModeRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2/task_state_mode",
      messageType: "std_msgs/msg/Int32MultiArray",
    });

    taskStateOdomResetRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2/task_state_odom_reset",
      messageType: "std_msgs/msg/Int32MultiArray",
    });

    taskAutoSendEnabledRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2/task_auto_send_enabled",
      messageType: "std_msgs/msg/Bool",
    });
    taskAutoSendEnabledRef.current.publish({ data: true });
    setPlannerAutoSendEnabled(true);

    taskStatusSubRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2/task_status",
      messageType: "std_msgs/msg/Int32MultiArray",
    });
    taskStatusSubRef.current.subscribe((msg) => {
      const data = Array.isArray(msg?.data) ? msg.data : [];
      if (data.length >= 1) {
        setPlannerStateCode(Number(data[0]) || 0);
      }
      if (data.length >= 2) {
        setPlannerColorCode(Number(data[1]));
      }
      if (data.length >= 3) {
        const nextCell = Number(data[2]);
        setPlannerCellCode(Number.isFinite(nextCell) ? nextCell : 0);
        setPlannerCellInput(String(Number.isFinite(nextCell) ? nextCell : 0));
      }
      if (data.length >= 4) {
        setPlannerTransitionModeCode(Number(data[3]) || 0);
      }
    });

    taskStatusTextSubRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2/task_status_text",
      messageType: "std_msgs/msg/String",
    });
    taskStatusTextSubRef.current.subscribe((msg) => {
      setPlannerStatusText(String(msg?.data || ""));
    });

    rosTopicsServiceRef.current = new ROSLIB.Service({
      ros: rosRef.current,
      name: "/rosapi/topics",
      serviceType: "rosapi_msgs/srv/Topics",
    });

    rosTopicTypeServiceRef.current = new ROSLIB.Service({
      ros: rosRef.current,
      name: "/rosapi/topic_type",
      serviceType: "rosapi_msgs/srv/TopicType",
    });

    // 10Hzで送信
    const interval = setInterval(() => {
      if (!controllerEnabled || !operationArmed) return;

      if (commandRef.current) {
        commandRef.current.publish({
          data: [commandValueRef.current],
        });
      }

      if (joyRef.current) {
        joyRef.current.publish({
          axes: axesRef.current,
          buttons: buttonsRef.current,
        });
      }
    }, 100);

    return () => {
      clearInterval(interval);
      // Cleanup joy topic explicitly before closing ROS
      if (joyRef.current) {
        try {
          joyRef.current.unsubscribe?.();
        } catch (error) {
          console.warn("Error unsubscribing joy topic:", error);
        }
      }
      virtualOdomPubRef.current = null;
      if (odomRef.current) {
        try {
          odomRef.current.unsubscribe?.();
        } catch (error) {
          console.warn("Error unsubscribing odom topic:", error);
        }
      }
      if (driveModeRef.current) {
        try {
          driveModeRef.current.unsubscribe?.();
        } catch (error) {
          console.warn("Error unsubscribing drive mode topic:", error);
        }
      }
      if (taskStatusSubRef.current) {
        try {
          taskStatusSubRef.current.unsubscribe?.();
        } catch (error) {
          console.warn("Error unsubscribing task status topic:", error);
        }
      }
      if (taskStatusTextSubRef.current) {
        try {
          taskStatusTextSubRef.current.unsubscribe?.();
        } catch (error) {
          console.warn("Error unsubscribing task status text topic:", error);
        }
      }
      if (arucoPoseSubRef.current) {
        try {
          arucoPoseSubRef.current.unsubscribe?.();
        } catch (error) {
          console.warn("Error unsubscribing aruco pose topic:", error);
        }
      }
      if (arucoIdSubRef.current) {
        try {
          arucoIdSubRef.current.unsubscribe?.();
        } catch (error) {
          console.warn("Error unsubscribing aruco id topic:", error);
        }
      }
      if (wallAngleSubRef.current) {
        try {
          wallAngleSubRef.current.unsubscribe?.();
        } catch (error) {
          console.warn("Error unsubscribing wall surface estimation topic:", error);
        }
      }
      try {
        filteredPointsTopic.unsubscribe?.();
      } catch (error) {
        console.warn("Error unsubscribing filtered points topic:", error);
      }
      try {
        ransacParamsTopic.unsubscribe?.();
      } catch (error) {
        console.warn("Error unsubscribing ransac params topic:", error);
      }
      try {
        cubeDetectedTopic.unsubscribe?.();
      } catch (error) {
        console.warn("Error unsubscribing cube detected topic:", error);
      }
      try {
        cubeInfoTopic.unsubscribe?.();
      } catch (error) {
        console.warn("Error unsubscribing cube info topic:", error);
      }
      stopCubeDebugStream();
      stopCameraStream();
      stopTopicEcho();
      if (rosRef.current) rosRef.current.close();
    };
  }, [rosUrl, joyTopicName, virtualOdomTopicName, wallAngleTopicName]);

  useEffect(() => {
    if (!virtualOdomEnabled || !virtualOdomPubRef.current) {
      if (virtualOdomTimerRef.current) {
        clearInterval(virtualOdomTimerRef.current);
        virtualOdomTimerRef.current = null;
      }
      return;
    }

    const hz = Math.max(1, Math.min(50, parseInt(virtualOdomHzInput, 10) || 10));
    const intervalMs = Math.max(20, Math.round(1000 / hz));
    const timer = setInterval(() => {
      if (!virtualOdomPubRef.current) return;
      virtualOdomPubRef.current.publish({
        data: [poseX, poseY, poseYaw],
      });
    }, intervalMs);

    virtualOdomTimerRef.current = timer;
    return () => {
      clearInterval(timer);
      virtualOdomTimerRef.current = null;
    };
  }, [virtualOdomEnabled, virtualOdomHzInput, poseX, poseY, poseYaw]);

  // Track if planner configuration is dirty
  useEffect(() => {
    setPlannerConfigIsDirty(true);
  }, [plannerStateSequence, plannerStatePoseConfig, plannerStateModeConfig, plannerStateOdomResetConfig, plannerCustomStates]);

  // Register beforeunload warning for unsaved changes
  useEffect(() => {
    const handleBeforeUnload = (event) => {
      if (!plannerConfigIsDirty) {
        return;
      }
      event.preventDefault();
      event.returnValue = tr(
        "未保存の設定変更があります。本当にページを閉じますか？",
        "You have unsaved configuration changes. Are you sure?"
      );
    };

    window.addEventListener("beforeunload", handleBeforeUnload);
    return () => window.removeEventListener("beforeunload", handleBeforeUnload);
  }, [plannerConfigIsDirty]);

  if (frontendForceStopped) {
    return (
      <div className="console-page">
        <div className="console-bg-shape console-bg-shape-a" />
        <div className="console-bg-shape console-bg-shape-b" />

        <main className="console-card">
          <header className="console-header">
            <img src="/logo.svg" alt="NR26 Logo" className="console-logo" />
            <div>
              <h1>R2 Console</h1>
              <p>{tr("フロントエンド強制停止", "Frontend Forced Shutdown")}</p>
              <p className="console-version-text">ver {GUI_VERSION}</p>
            </div>
          </header>

          <section className="disabled-notice">
            {tr("フロントエンドを強制停止しました。復帰するには再読み込みしてください。", "Frontend is force-stopped. Reload to resume.")}
          </section>

          <section className="control-toggle-row">
            <button className="connection-button btn-connect" onClick={() => window.location.reload()}>
              {tr("再読み込み", "Reload")}
            </button>
          </section>
        </main>
      </div>
    );
  }

  if (controllerFullscreen) {
    return (
      <div className="console-page console-page-fullscreen">
        <main className="console-card console-card-fullscreen-controller">
          <button
            className="fullscreen-close-button-top"
            onClick={() => setControllerFullscreen(false)}
          >
            {tr("✕ 通常表示に戻る", "✕ Back to Normal")}
          </button>

          <div className="ps4-panel-fullscreen">
            <div className="ps-shoulder-row">
              <div className="ps-shoulder-side">
                <button className={`ps-button ps-shoulder ${buttons[6] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(6)}>
                  L2
                </button>
                <button className={`ps-button ps-shoulder ${buttons[4] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(4)}>
                  L1
                </button>
              </div>
              <div className="ps-shoulder-side ps-shoulder-side-right">
                <button className={`ps-button ps-shoulder ${buttons[5] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(5)}>
                  R1
                </button>
                <button className={`ps-button ps-shoulder ${buttons[7] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(7)}>
                  R2
                </button>
              </div>
            </div>

            <div className="ps-main-row-fullscreen">
              <div className="dpad-grid-fullscreen">
                <button className={`dpad-button ${axes[7] === 1 ? "ps-active" : ""}`} {...getAxisPressProps(7, 1)}>
                  ↑
                </button>
                <button className={`dpad-button ${axes[6] === -1 ? "ps-active" : ""}`} {...getAxisPressProps(6, -1)}>
                  ←
                </button>
                <button className={`dpad-button ${axes[6] === 1 ? "ps-active" : ""}`} {...getAxisPressProps(6, 1)}>
                  →
                </button>
                <button className={`dpad-button ${axes[7] === -1 ? "ps-active" : ""}`} {...getAxisPressProps(7, -1)}>
                  ↓
                </button>
              </div>

              <div className="face-grid-fullscreen">
                <button className={`ps-button ps-face ps-triangle ${buttons[2] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(2)}>
                  △
                </button>
                <button className={`ps-button ps-face ps-square ${buttons[3] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(3)}>
                  □
                </button>
                <button className={`ps-button ps-face ps-circle ${buttons[1] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(1)}>
                  ○
                </button>
                <button className={`ps-button ps-face ps-cross ${buttons[0] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(0)}>
                  ×
                </button>
              </div>
            </div>

            <div className="ps-system-row-fullscreen">
              <button className={`ps-button ps-system ${buttons[8] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(8)}>
                SHARE
              </button>
              <button className={`ps-button ps-system ps-home ${buttons[12] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(12)}>
                PS
              </button>
              <button className={`ps-button ps-system ${buttons[9] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(9)}>
                OPTIONS
              </button>
            </div>

            <div className="ps-stick-row-fullscreen">
              <button className={`ps-button ps-stick ${buttons[10] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(10)}>
                L3
              </button>
              <button className={`ps-button ps-stick ${buttons[11] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(11)}>
                R3
              </button>
            </div>
          </div>
        </main>
      </div>
    );
  }

  return (
    <div className="console-page">
      <div className="console-bg-shape console-bg-shape-a" />
      <div className="console-bg-shape console-bg-shape-b" />

      <main className="console-card">
        <header className="console-header">
          <img src="/logo.svg" alt="NR26 Logo" className="console-logo" />
          <div>
            <h1>R2 Console</h1>
            <p>{tr("R2 テレオペレーションパネル", "ROS2 Teleoperation Panel")}</p>
            <p className="console-version-text">ver {GUI_VERSION}</p>
          </div>
          <div className="header-tools">
            {renderLanguageSelect("lang-select", "lang-header")}
          </div>
        </header>

        <section className="connection-status-bar">
          <div className="connection-status-top">
            <div className="connection-status-inputs">
              <input
                className="connection-input"
                value={rosHostInput}
                onChange={(e) => setRosHostInput(e.target.value)}
                onKeyDown={(e) => {
                  if (e.key === "Enter") applyRosEndpoint();
                }}
                placeholder={tr("ROS IP (例: 192.168.1.10)", "ROS Host (e.g. 192.168.1.10)")}
              />
              <input
                className="connection-input connection-port"
                value={rosPortInput}
                onChange={(e) => setRosPortInput(e.target.value)}
                onKeyDown={(e) => {
                  if (e.key === "Enter") applyRosEndpoint();
                }}
                placeholder="9090"
              />
            </div>
            <button className="connection-button btn-connect" onClick={applyRosEndpoint}>
              {tr("接続更新", "Apply")}
            </button>
          </div>

          <div className="connection-status-meta">
            <span className="connection-compact-url">{rosUrl}</span>
            <span
              className={`status-pill ${status === "接続OK"
                ? "status-ok"
                : status === "接続中..."
                  ? "status-pending"
                  : "status-bad"
                }`}
            >
              {localizedStatusText}
            </span>
          </div>
        </section>

        <section className="control-toggle-row">
          <button
            className={`toggle-button ${operationArmed ? "toggle-on" : "toggle-off"}`}
            onClick={() => {
              const next = !operationArmed;
              setOperationArmed(next);
              if (!next) {
                resetAllControls();
              }
            }}
          >
            {operationArmed ? tr("操作ロック: OFF", "Safety Lock: OFF") : tr("操作ロック: ON", "Safety Lock: ON")}
          </button>
          <button
            className={`toggle-button page-mode-toggle-button ${multiTabMode ? "toggle-on" : "toggle-off"}`}
            onClick={() => setMultiTabMode((prev) => !prev)}
          >
            {multiTabMode ? tr("タブモード: 複数", "Tab Mode: Multi") : tr("タブモード: 単一", "Tab Mode: Single")}
          </button>
        </section>
        <p className={`lock-notice ${operationArmed ? "lock-notice-on" : "lock-notice-off"}`}>
          {lockNoticeText}
        </p>
        <p className="connection-hint page-mode-hint">
          {multiTabMode
            ? tr("複数タブモードでは最大2タブを同時表示します", "Multi-tab mode shows up to 2 tabs at once")
            : tr("単一タブモードでは常に1タブのみ表示します", "Single-tab mode always shows one tab")}
        </p>

        <section className="page-switch-row">
          {pageOrder.map((page) => (
            <button
              key={`page-tab-${page}`}
              className={`page-switch-button ${isPageActive(page) ? "page-switch-active" : ""}`}
              onClick={() => handlePageButtonClick(page)}
              disabled={multiTabMode && !isPageActive(page) && activePages.length >= MAX_ACTIVE_PAGES}
              title={
                multiTabMode && !isPageActive(page) && activePages.length >= MAX_ACTIVE_PAGES
                  ? tr("同時表示は2タブまでです", "Up to 2 tabs can be shown at once")
                  : multiTabMode
                    ? tr("複数タブモード: 最大2タブまで同時表示", "Multi-tab mode: up to 2 tabs at once")
                    : tr("単一タブモード: クリックしたタブに切替", "Single-tab mode: click to switch tabs")
              }
            >
              {getPageLabel(page)}
            </button>
          ))}
        </section>
        <p className="connection-hint page-switch-hint">
          {multiTabMode
            ? tr("同時に表示できるタブは最大2つです", "You can show up to 2 tabs at the same time")
            : tr("単一タブモードでは複数同時表示はできません", "In single-tab mode, multiple tabs cannot be shown")}
        </p>

        <section className="active-pages-grid">

          {isPageActive("controller") && (
            <section className="serial-bridge-panel">
              <section className="joy-topic-row">
                <input
                  className="connection-input"
                  value={joyTopicInput}
                  onChange={(e) => setJoyTopicInput(e.target.value)}
                  onKeyDown={(e) => {
                    if (e.key === "Enter") applyJoyTopicName();
                  }}
                  placeholder={tr("Joy Topic Name (例: /joy_9)", "Joy Topic Name (e.g. /joy_9)")}
                />
                <button className="connection-button btn-connect" onClick={applyJoyTopicName}>
                  {tr("更新", "Apply")}
                </button>
              </section>

              <p className="connection-hint">{tr("現在のJoyトピック:", "Current Joy topic:")} {joyTopicName}</p>

              <section className="control-toggle-row">
                <button
                  className={`toggle-button ${controllerEnabled ? "toggle-on" : "toggle-off"}`}
                  onClick={() => setControllerEnabled(!controllerEnabled)}
                >
                  {controllerEnabled ? tr("コントローラー: ON", "Controller: ON") : tr("コントローラー: OFF", "Controller: OFF")}
                </button>
                <button
                  className="fullscreen-toggle-button"
                  onClick={() => setControllerFullscreen(true)}
                >
                  {tr("全画面操作", "Fullscreen")}
                </button>
              </section>

              {controllerEnabled && (
                <div className="controller-layout">
                  <section className="quick-controls-panel">
                    <div className="quick-controls-row">
                      <div className="control-group">
                        <button
                          className="control-button control-button-emergency emergency-stop-shape"
                          onClick={() => {
                            updateCommand(0);
                          }}
                        >
                          {tr("緊急停止", "Emergency Stop")}
                        </button>
                      </div>
                    </div>

                    <p className="velocity-readout">
                      command: {commandValue}
                    </p>
                  </section>

                  <section className="ps4-panel">
                    <h2>{tr("PS4 コントローラーボタン", "PS4 Controller Buttons")}</h2>

                    <div className="ps-shoulder-row">
                      <div className="ps-shoulder-side">
                        <button className={`ps-button ps-shoulder ${buttons[6] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(6)}>
                          L2
                        </button>
                        <button className={`ps-button ps-shoulder ${buttons[4] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(4)}>
                          L1
                        </button>
                      </div>
                      <div className="ps-shoulder-side ps-shoulder-side-right">
                        <button className={`ps-button ps-shoulder ${buttons[5] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(5)}>
                          R1
                        </button>
                        <button className={`ps-button ps-shoulder ${buttons[7] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(7)}>
                          R2
                        </button>
                      </div>
                    </div>

                    <div className="ps-main-row">
                      <div className="dpad-grid">
                        <button className={`dpad-button ${axes[7] === 1 ? "ps-active" : ""}`} {...getAxisPressProps(7, 1)}>
                          ↑
                        </button>
                        <button className={`dpad-button ${axes[6] === -1 ? "ps-active" : ""}`} {...getAxisPressProps(6, -1)}>
                          ←
                        </button>
                        <button className={`dpad-button ${axes[6] === 1 ? "ps-active" : ""}`} {...getAxisPressProps(6, 1)}>
                          →
                        </button>
                        <button className={`dpad-button ${axes[7] === -1 ? "ps-active" : ""}`} {...getAxisPressProps(7, -1)}>
                          ↓
                        </button>
                      </div>

                      <div className="ps-system-row">
                        <button className={`ps-button ps-system ${buttons[8] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(8)}>
                          SHARE
                        </button>
                        <button className={`ps-button ps-system ps-home ${buttons[12] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(12)}>
                          PS
                        </button>
                        <button className={`ps-button ps-system ${buttons[9] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(9)}>
                          OPTIONS
                        </button>
                      </div>

                      <div className="face-grid">
                        <button className={`ps-button ps-face ps-triangle ${buttons[2] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(2)}>
                          △
                        </button>
                        <button className={`ps-button ps-face ps-square ${buttons[3] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(3)}>
                          □
                        </button>
                        <button className={`ps-button ps-face ps-circle ${buttons[1] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(1)}>
                          ○
                        </button>
                        <button className={`ps-button ps-face ps-cross ${buttons[0] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(0)}>
                          ×
                        </button>
                      </div>
                    </div>

                    <div className="ps-stick-row">
                      <button className={`ps-button ps-stick ${buttons[10] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(10)}>
                        L3
                      </button>
                      <button className={`ps-button ps-stick ${buttons[11] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(11)}>
                        R3
                      </button>
                    </div>
                  </section>
                </div>
              )}

              {!controllerEnabled && (
                <div className="disabled-notice">
                  {tr("コントローラーは無効化されています", "Controller is disabled")}
                </div>
              )}
            </section>
          )}

          {isPageActive("pose") && (
            <section className="pose-panel">
              <h2 className="serial-packet-title">{tr("座標・姿勢管理", "Pose Manager")}</h2>
              <p className="serial-packet-hint">
                {tr(
                  "現在位置と目標値を確認し、必要に応じて目標値を送信します。",
                  "Check current pose and target values, then publish targets when needed."
                )}
              </p>

              <div className="pose-overview-grid">
                <section className="pose-graph-card">
                  <div className="pose-graph-title-row">
                    <h3 className="pose-graph-title">{tr("座標グラフ", "Pose Graph")}</h3>
                    <span className="pose-graph-scale">{tr("単位: m", "Unit: m")}</span>
                  </div>

                  <svg
                    className="pose-graph"
                    ref={poseGraphRef}
                    viewBox={`0 0 ${graphWidth} ${graphHeight}`}
                    role="img"
                    aria-label={tr("座標・姿勢管理グラフ", "Pose management graph")}
                    onPointerDown={handlePoseGraphPointerDown}
                    onPointerMove={handlePoseGraphPointerMove}
                    onPointerUp={releasePoseGraphPointer}
                    onPointerCancel={releasePoseGraphPointer}
                    onPointerLeave={releasePoseGraphPointer}
                  >
                    <rect
                      x={graphPadding}
                      y={graphPadding}
                      width={graphInnerWidth}
                      height={graphInnerHeight}
                      className="pose-graph-frame"
                    />

                    {gridXValues.map((value) => (
                      <line
                        key={`pose-grid-x-${value}`}
                        x1={toGraphX(value)}
                        y1={graphPadding}
                        x2={toGraphX(value)}
                        y2={graphHeight - graphPadding}
                        className="pose-graph-grid"
                      />
                    ))}
                    {gridYValues.map((value) => (
                      <line
                        key={`pose-grid-y-${value}`}
                        x1={graphPadding}
                        y1={toGraphY(value)}
                        x2={graphWidth - graphPadding}
                        y2={toGraphY(value)}
                        className="pose-graph-grid"
                      />
                    ))}

                    {axisXVisible && (
                      <line
                        x1={graphPadding}
                        y1={toGraphY(0)}
                        x2={graphWidth - graphPadding}
                        y2={toGraphY(0)}
                        className="pose-graph-axis"
                      />
                    )}
                    {axisYVisible && (
                      <line
                        x1={toGraphX(0)}
                        y1={graphPadding}
                        x2={toGraphX(0)}
                        y2={graphHeight - graphPadding}
                        className="pose-graph-axis"
                      />
                    )}

                    <line x1={currentX} y1={currentY} x2={targetX} y2={targetY} className="pose-graph-link" />
                    <line x1={currentX} y1={currentY} x2={currentArrowX} y2={currentArrowY} className="pose-graph-arrow-current" />
                    <line x1={targetX} y1={targetY} x2={targetArrowX} y2={targetArrowY} className="pose-graph-arrow-target" />

                    <circle cx={currentX} cy={currentY} r="6" className="pose-graph-point-current" />
                    <circle cx={targetX} cy={targetY} r="6" className="pose-graph-point-target" />
                    {arucoDetectedX !== null && arucoDetectedY !== null && (
                      <circle cx={arucoDetectedX} cy={arucoDetectedY} r="5" className="pose-graph-point-aruco" />
                    )}
                    {arucoTargetX !== null && arucoTargetY !== null && (
                      <circle cx={arucoTargetX} cy={arucoTargetY} r="5" className="pose-graph-point-aruco-target" />
                    )}

                    <text x={currentX + 8} y={currentY - 8} className="pose-graph-label">{tr("現在", "Current")}</text>
                    <text x={targetX + 8} y={targetY - 8} className="pose-graph-label">{tr("目標", "Target")}</text>
                    {arucoDetectedX !== null && arucoDetectedY !== null && (
                      <text x={arucoDetectedX + 8} y={arucoDetectedY - 8} className="pose-graph-label">
                        {arucoDetectedId !== null
                          ? tr(`ArUco(ID:${arucoDetectedId})`, `ArUco(ID:${arucoDetectedId})`)
                          : tr("ArUco", "ArUco")}
                      </text>
                    )}
                    {arucoTargetX !== null && arucoTargetY !== null && (
                      <text x={arucoTargetX + 8} y={arucoTargetY + 14} className="pose-graph-label">
                        {tr("ArUco目標", "ArUco Target")}
                      </text>
                    )}

                    <text x={graphPadding} y={graphPadding - 8} className="pose-graph-corner-label">
                      Y {graphMaxY.toFixed(2)}
                    </text>
                    <text x={graphPadding} y={graphHeight - 8} className="pose-graph-corner-label">
                      Y {graphMinY.toFixed(2)}
                    </text>
                    <text x={graphPadding} y={graphHeight - graphPadding + 16} className="pose-graph-corner-label">
                      X {graphMinX.toFixed(2)}
                    </text>
                    <text x={graphWidth - graphPadding - 84} y={graphHeight - graphPadding + 16} className="pose-graph-corner-label">
                      X {graphMaxX.toFixed(2)}
                    </text>
                  </svg>

                  <div className="pose-graph-legend">
                    <span className="pose-legend-item">
                      <i className="pose-legend-dot pose-legend-current" />
                      {tr("現在位置", "Current position")}
                    </span>
                    <span className="pose-legend-item">
                      <i className="pose-legend-dot pose-legend-target" />
                      {tr("目標位置", "Target position")}
                    </span>
                    <span className="pose-legend-item">
                      <i className="pose-legend-dot pose-legend-aruco" />
                      {tr("検出ArUco", "Detected ArUco")}
                    </span>
                    <span className="pose-legend-item">
                      <i className="pose-legend-dot pose-legend-aruco-target" />
                      {tr("ArUco目標点", "ArUco target point")}
                    </span>
                  </div>

                  <p className="pose-graph-interaction-hint">
                    {tr(
                      "グラフをドラッグして目標X/Yを更新できます。Yawは入力欄で調整してください。",
                      "Drag on the graph to update target X/Y. Adjust yaw using the input field."
                    )}
                  </p>
                </section>

                <div className="pose-current-grid">
                  <div className="pose-current-item">
                    <span>{tr("現在X", "Current X")}</span>
                    <strong>{poseX.toFixed(3)}</strong>
                  </div>
                  <div className="pose-current-item">
                    <span>{tr("現在Y", "Current Y")}</span>
                    <strong>{poseY.toFixed(3)}</strong>
                  </div>
                  <div className="pose-current-item">
                    <span>{tr("現在Yaw (°)", "Current Yaw (°)")}</span>
                    <strong>{(poseYaw * 180 / Math.PI).toFixed(1)}</strong>
                  </div>
                  <div className="pose-current-item">
                    <span>{tr("現在モード", "Current Mode")}</span>
                    <strong>{driveMode}</strong>
                  </div>
                </div>
              </div>

              <div className="pose-step-grid">
                <label className="serial-packet-label">
                  {tr("X ステップ", "X Step")}
                  <input
                    className="connection-input"
                    type="number"
                    step="0.1"
                    value={targetXStep}
                    onChange={(e) => setTargetXStep(e.target.value)}
                  />
                </label>
                <label className="serial-packet-label">
                  {tr("Y ステップ", "Y Step")}
                  <input
                    className="connection-input"
                    type="number"
                    step="0.1"
                    value={targetYStep}
                    onChange={(e) => setTargetYStep(e.target.value)}
                  />
                </label>
                <label className="serial-packet-label">
                  {tr("Yaw ステップ (°)", "Yaw Step (°)")}
                  <input
                    className="connection-input"
                    type="number"
                    step="1"
                    value={targetYawStep}
                    onChange={(e) => setTargetYawStep(e.target.value)}
                  />
                </label>
              </div>

              <div className="pose-input-grid">
                <div className="pose-input-item">
                  <label className="serial-packet-label">
                    {tr("目標X", "Target X")}
                    <input className="connection-input" value={targetXInput} onChange={(e) => setTargetXInput(e.target.value)} />
                  </label>
                  <div className="pose-button-group">
                    <button className="pose-pm-button" onClick={() => decrementTarget(setTargetXInput, targetXInput, targetXStep)}>−</button>
                    <button className="pose-pm-button" onClick={() => incrementTarget(setTargetXInput, targetXInput, targetXStep)}>+</button>
                  </div>
                </div>

                <div className="pose-input-item">
                  <label className="serial-packet-label">
                    {tr("目標Y", "Target Y")}
                    <input className="connection-input" value={targetYInput} onChange={(e) => setTargetYInput(e.target.value)} />
                  </label>
                  <div className="pose-button-group">
                    <button className="pose-pm-button" onClick={() => decrementTarget(setTargetYInput, targetYInput, targetYStep)}>−</button>
                    <button className="pose-pm-button" onClick={() => incrementTarget(setTargetYInput, targetYInput, targetYStep)}>+</button>
                  </div>
                </div>

                <div className="pose-input-item">
                  <label className="serial-packet-label">
                    {tr("目標Yaw (°)", "Target Yaw (°)")}
                    <input className="connection-input" value={targetYawInput} onChange={(e) => setTargetYawInput(e.target.value)} />
                  </label>
                  <div className="pose-button-group">
                    <button className="pose-pm-button" onClick={() => decrementTarget(setTargetYawInput, targetYawInput, targetYawStep)}>−</button>
                    <button className="pose-pm-button" onClick={() => incrementTarget(setTargetYawInput, targetYawInput, targetYawStep)}>+</button>
                  </div>
                </div>
              </div>

              <div className="pose-actions-row">
                <button className="connection-button btn-neutral" onClick={applyAutoDriveFromCurrentPose}>{tr("現在値を目標へ", "Use Current as Target")}</button>
                <button className="connection-button btn-neutral" onClick={publishOdomReset} disabled={!operationArmed}>{tr("オドメトリをリセット", "Reset Odometry")}</button>
                <button className="connection-button serial-send-button btn-send" onClick={publishAutoDriveCommand} disabled={!operationArmed}>{tr("目標座標を送信", "Send Target Pose")}</button>
              </div>

              <p className="connection-hint">{translateRuntimeText(autoDriveCmdInfo)}</p>

              <section className={`aruco-control-panel ${driveMode === "ARUCO" ? "aruco-control-panel-active" : ""}`}>
                <div className="aruco-control-header">
                  <h3 className="aruco-control-title">{tr("ArUcoモード設定", "ArUco Mode Settings")}</h3>
                  <span className="aruco-control-badge">{driveMode}</span>
                </div>
                <p className="aruco-control-description">
                  {tr(
                    "一旦は奥行きと姿勢を無視し、カメラの横位置だけを合わせます。カメラが左向きなので、これは機体の前後移動に対応します。",
                    "For now, ignore depth and orientation and align only the camera horizontal offset. Because the camera faces left, this corresponds to robot forward/back motion."
                  )}
                </p>

                <div className="aruco-control-grid">
                  <div className="pose-input-item">
                    <label className="serial-packet-label">
                      {tr("追尾ID (-1 で全て)", "Target ID (-1 for any)")}
                      <input
                        className="connection-input"
                        value={arucoTargetIdInput}
                        onChange={(e) => setArucoTargetIdInput(e.target.value)}
                      />
                    </label>
                  </div>

                  <div className="pose-input-item">
                    <label className="serial-packet-label">
                      {tr("横位置目標 (m)", "Horizontal Offset Target (m)")}
                      <input
                        className="connection-input"
                        value={arucoTargetForwardInput}
                        onChange={(e) => setArucoTargetForwardInput(e.target.value)}
                      />
                    </label>
                  </div>

                  <div className="pose-input-item">
                    <label className="serial-packet-label">
                      {tr("奥行き目標 (未使用)", "Depth Target (unused)")}
                      <input
                        className="connection-input"
                        value={arucoTargetLateralInput}
                        disabled
                        onChange={(e) => setArucoTargetLateralInput(e.target.value)}
                      />
                    </label>
                  </div>

                  <div className="pose-input-item">
                    <label className="serial-packet-label">
                      {tr("姿勢目標 (未使用)", "Yaw Target (unused)")}
                      <input
                        className="connection-input"
                        value={arucoTargetYawInput}
                        disabled
                        onChange={(e) => setArucoTargetYawInput(e.target.value)}
                      />
                    </label>
                  </div>

                  <div className="pose-input-item">
                    <label className="serial-packet-label">
                      {tr("カメラ位置 X (m)", "Camera Position X (m)")}
                      <input
                        className="connection-input"
                        value={arucoCameraOffsetXInput}
                        onChange={(e) => setArucoCameraOffsetXInput(e.target.value)}
                      />
                    </label>
                  </div>

                  <div className="pose-input-item">
                    <label className="serial-packet-label">
                      {tr("カメラ位置 Y (m)", "Camera Position Y (m)")}
                      <input
                        className="connection-input"
                        value={arucoCameraOffsetYInput}
                        onChange={(e) => setArucoCameraOffsetYInput(e.target.value)}
                      />
                    </label>
                  </div>
                </div>

                <div className="pose-actions-row">
                  <button className="connection-button btn-neutral" onClick={publishArucoTargetId} disabled={!operationArmed}>
                    {tr("追尾IDを送信", "Send Target ID")}
                  </button>
                  <button className="connection-button serial-send-button btn-send" onClick={publishArucoTargetCommand} disabled={!operationArmed}>
                    {tr("横位置目標を送信", "Send Horizontal Target")}
                  </button>
                  <button className="connection-button btn-neutral" onClick={publishArucoCameraOffset} disabled={!operationArmed}>
                    {tr("カメラ位置を送信", "Send Camera Position")}
                  </button>
                </div>

                <p className="connection-hint">{translateRuntimeText(arucoCmdInfo)}</p>
              </section>

              <div className="pose-target-save-panel">
                <button className="connection-button serial-send-button btn-save" onClick={saveTargetPose}>
                  {tr("目標値を保存", "Save Target")}
                </button>
                {savedPosesList.length > 0 && (
                  <button className="serial-clear-button" onClick={clearAllSavedPoses}>
                    {tr("すべてクリア", "Clear All")}
                  </button>
                )}
              </div>

              {savedPosesList.length > 0 && (
                <div className="pose-saved-list-panel">
                  <h3 className="pose-saved-list-title">{tr("保存済み目標値", "Saved Targets")} ({savedPosesList.length})</h3>
                  <div className="pose-saved-list">
                    {savedPosesList.map((pose) => (
                      <div key={pose.id} className="pose-saved-item">
                        <div className="pose-saved-item-info">
                          <span className="pose-saved-item-label">{pose.label}</span>
                          <span className="pose-saved-item-values">
                            X: {pose.x.toFixed(3)}, Y: {pose.y.toFixed(3)}, Yaw: {pose.yawDeg.toFixed(1)}°
                          </span>
                          <span className="pose-saved-item-time">{pose.timestamp}</span>
                        </div>
                        <div className="pose-saved-item-actions">
                          <button
                            className="connection-button pose-item-button btn-restore"
                            onClick={() => applySavedTargetPose(pose)}
                          >
                            {tr("復元&送信", "Restore & Send")}
                          </button>
                          <button
                            className="serial-clear-button pose-item-button"
                            onClick={() => deleteSavedPose(pose.id)}
                          >
                            {tr("削除", "Delete")}
                          </button>
                        </div>
                      </div>
                    ))}
                  </div>
                </div>
              )}
            </section>
          )}

          {isPageActive("wall-angle") && (
            <section className="pose-panel">
              <h2 className="serial-packet-title">{tr("壁面推定の図示", "Wall Surface Estimation Visualization")}</h2>
              <p className="serial-packet-hint">
                {tr(
                  "wall_detection の角度トピックを購読して、現在の壁面推定結果をゲージ表示します。",
                  "Subscribes to wall_detection angle topic and displays current wall surface estimation as a gauge."
                )}
              </p>

              <section className="joy-topic-row">
                <input
                  className="connection-input"
                  value={wallAngleTopicInput}
                  onChange={(e) => setWallAngleTopicInput(e.target.value)}
                  onKeyDown={(e) => {
                    if (e.key === "Enter") applyWallAngleTopicName();
                  }}
                  placeholder="/wall_detection/angle"
                />
                <button className="connection-button btn-connect" onClick={applyWallAngleTopicName}>
                  {tr("更新", "Apply")}
                </button>
              </section>

              <p className="connection-hint">{tr("現在の購読トピック:", "Current topic:")} {wallAngleTopicName}</p>

              <div className="pose-overview-grid">
                <section className="pose-graph-card">
                  <div className="pose-graph-title-row">
                    <h3 className="pose-graph-title">{tr("壁面推定グラフ", "Wall Surface Estimation Graph")}</h3>
                    <span className="pose-graph-scale">{tr("単位: m", "Unit: m")}</span>
                  </div>
                  <svg
                    className="wall-angle-graph"
                    viewBox={`0 0 ${wallGraphWidth} ${wallGraphHeight}`}
                    role="img"
                    aria-label={tr("壁面推定グラフ", "Wall surface estimation graph")}
                    style={{ backgroundColor: "#ffffff" }}
                  >
                    {/* Background */}
                    <rect
                      x="0"
                      y="0"
                      width={wallGraphWidth}
                      height={wallGraphHeight}
                      fill="white"
                      stroke="none"
                    />

                    {/* Grid lines */}
                    {wallGridXValues.map((xVal) => (
                      <line
                        key={`wall-grid-x-${xVal}`}
                        x1={toWallGraphX(xVal)}
                        y1={wallGraphPadding}
                        x2={toWallGraphX(xVal)}
                        y2={wallGraphHeight - wallGraphPadding}
                        stroke="#d0d0d0"
                        strokeWidth="1"
                        strokeDasharray="2,2"
                      />
                    ))}
                    {wallGridYValues.map((yVal) => (
                      <line
                        key={`wall-grid-y-${yVal}`}
                        x1={wallGraphPadding}
                        y1={toWallGraphY(yVal)}
                        x2={wallGraphWidth - wallGraphPadding}
                        y2={toWallGraphY(yVal)}
                        stroke="#d0d0d0"
                        strokeWidth="1"
                        strokeDasharray="2,2"
                      />
                    ))}

                    {/* Axis lines */}
                    {wallAxisXVisible && (
                      <line
                        x1={wallGraphPadding}
                        y1={toWallGraphY(0)}
                        x2={wallGraphWidth - wallGraphPadding}
                        y2={toWallGraphY(0)}
                        stroke="#333333"
                        strokeWidth="1.5"
                      />
                    )}
                    {wallAxisYVisible && (
                      <line
                        x1={toWallGraphX(0)}
                        y1={wallGraphPadding}
                        x2={toWallGraphX(0)}
                        y2={wallGraphHeight - wallGraphPadding}
                        stroke="#333333"
                        strokeWidth="1.5"
                      />
                    )}

                    {/* RANSAC line */}
                    {ransacLinePoint1 && ransacLinePoint2 && (
                      <line
                        x1={toWallGraphX(ransacLinePoint1.x)}
                        y1={toWallGraphY(ransacLinePoint1.y)}
                        x2={toWallGraphX(ransacLinePoint2.x)}
                        y2={toWallGraphY(ransacLinePoint2.y)}
                        stroke="#e63946"
                        strokeWidth="2.5"
                      />
                    )}

                    {/* Filtered points */}
                    {wallFilteredPoints.map((point, idx) => {
                      const rotatedPoint = { x: -point.y, y: point.x };
                      const isInlier =
                        wallRansacParams && wallRansacParams.a !== 0 && wallRansacParams.b !== 0 &&
                        Math.abs(
                          wallRansacParams.a * point.x +
                          wallRansacParams.b * point.y +
                          wallRansacParams.c
                        ) < 0.1;

                      return (
                        <circle
                          key={`wall-point-${idx}`}
                          cx={toWallGraphX(rotatedPoint.x)}
                          cy={toWallGraphY(rotatedPoint.y)}
                          r="3"
                          fill={isInlier ? "rgba(255, 100, 100, 0.9)" : "rgba(100, 150, 255, 0.8)"}
                          stroke={isInlier ? "#c81414" : "#0064c8"}
                          strokeWidth="1"
                        />
                      );
                    })}

                    {/* Graph border */}
                    <rect
                      x={wallGraphPadding}
                      y={wallGraphPadding}
                      width={wallGraphInnerWidth}
                      height={wallGraphInnerHeight}
                      stroke="#666666"
                      strokeWidth="1.5"
                      fill="none"
                    />
                  </svg>
                </section>

                <section className="pose-detail-panel">
                  <h3 className="pose-detail-title">{tr("壁面推定情報", "Wall Surface Estimation Info")}</h3>
                  <div className="pose-current-grid">
                    <div className="pose-current-item">
                      <span>{tr("角度 [rad]", "Angle [rad]")}</span>
                      <strong>{wallAngleRad.toFixed(4)}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("角度 [deg]", "Angle [deg]")}</span>
                      <strong>{wallAngleDeg.toFixed(2)}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("検出点数", "Detected Points")}</span>
                      <strong>{wallFilteredPoints.length}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("インライア数", "Inliers")}</span>
                      <strong>{wallRansacParams?.inliers || 0} / {wallRansacParams?.total || 0}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("直線係数 a", "Line Param a")}</span>
                      <strong>{wallRansacParams?.a?.toFixed(4) || "0.0000"}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("直線係数 b", "Line Param b")}</span>
                      <strong>{wallRansacParams?.b?.toFixed(4) || "0.0000"}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("直線係数 c", "Line Param c")}</span>
                      <strong>{wallRansacParams?.c?.toFixed(4) || "0.0000"}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("更新時刻", "Updated")}</span>
                      <strong>{wallAngleUpdatedAt || "-"}</strong>
                    </div>
                  </div>
                </section>
              </div>
            </section>
          )}

          {isPageActive("waypoint") && (
            <section className="waypoint-panel">
              <h2 className="serial-packet-title">{tr("ウェイポイント指定", "Waypoint Planner")}</h2>
              <p className="serial-packet-hint">
                {tr(
                  "グラフをクリックしてウェイポイントを追加します。到達判定しきい値を満たすと、次のウェイポイントへ自動で目標値を切り替えます。",
                  "Click the graph to add waypoints. When reach thresholds are satisfied, target values switch to the next waypoint automatically."
                )}
              </p>

              <div className="waypoint-toolbar">
                <button className="connection-button btn-connect" onClick={startWaypointNavigation}>
                  {tr("追従開始", "Start")}
                </button>
                <button className="connection-button btn-neutral" onClick={stopWaypointNavigation}>
                  {tr("追従停止", "Stop")}
                </button>
                <button className="serial-clear-button" onClick={clearWaypoints}>
                  {tr("全削除", "Clear All")}
                </button>
                <button
                  className={`toggle-button ${waypointAutoAdvanceEnabled ? "toggle-on" : "toggle-off"}`}
                  onClick={() => setWaypointAutoAdvanceEnabled((prev) => !prev)}
                >
                  {waypointAutoAdvanceEnabled ? tr("自動切替: ON", "Auto Advance: ON") : tr("自動切替: OFF", "Auto Advance: OFF")}
                </button>
                <button
                  className={`toggle-button ${waypointAutoPublishEnabled ? "toggle-on" : "toggle-off"}`}
                  onClick={() => setWaypointAutoPublishEnabled((prev) => !prev)}
                >
                  {waypointAutoPublishEnabled ? tr("切替時送信: ON", "Publish on Switch: ON") : tr("切替時送信: OFF", "Publish on Switch: OFF")}
                </button>
              </div>

              <div className="waypoint-settings-grid">
                <label className="serial-packet-label">
                  {tr("到達距離しきい値 [m]", "Distance Threshold [m]")}
                  <input
                    className="connection-input"
                    type="number"
                    step="0.01"
                    min="0.01"
                    value={waypointDistanceThresholdInput}
                    onChange={(e) => setWaypointDistanceThresholdInput(e.target.value)}
                  />
                </label>

                <button
                  className={`toggle-button ${waypointCheckYaw ? "toggle-on" : "toggle-off"}`}
                  onClick={() => setWaypointCheckYaw((prev) => !prev)}
                >
                  {waypointCheckYaw ? tr("Yaw判定: ON", "Yaw Check: ON") : tr("Yaw判定: OFF", "Yaw Check: OFF")}
                </button>

                <label className="serial-packet-label">
                  {tr("Yawしきい値 [deg]", "Yaw Threshold [deg]")}
                  <input
                    className="connection-input"
                    type="number"
                    step="0.5"
                    min="0.5"
                    value={waypointYawThresholdInput}
                    onChange={(e) => setWaypointYawThresholdInput(e.target.value)}
                    disabled={!waypointCheckYaw}
                  />
                </label>
              </div>

              <div className="waypoint-overview-grid">
                <section className="pose-graph-card">
                  <div className="pose-graph-title-row">
                    <h3 className="pose-graph-title">{tr("ウェイポイントグラフ", "Waypoint Graph")}</h3>
                    <span className="pose-graph-scale">{tr("単位: m", "Unit: m")}</span>
                  </div>

                  <svg
                    className="pose-graph waypoint-graph"
                    ref={waypointGraphRef}
                    viewBox={`0 0 ${graphWidth} ${graphHeight}`}
                    role="img"
                    aria-label={tr("ウェイポイント指定グラフ", "Waypoint plot graph")}
                    onPointerDown={handleWaypointGraphPointerDown}
                  >
                    <rect
                      x={graphPadding}
                      y={graphPadding}
                      width={graphInnerWidth}
                      height={graphInnerHeight}
                      className="pose-graph-frame"
                    />

                    {gridXValues.map((value) => (
                      <line
                        key={`wp-grid-x-${value}`}
                        x1={toGraphX(value)}
                        y1={graphPadding}
                        x2={toGraphX(value)}
                        y2={graphHeight - graphPadding}
                        className="pose-graph-grid"
                      />
                    ))}
                    {gridYValues.map((value) => (
                      <line
                        key={`wp-grid-y-${value}`}
                        x1={graphPadding}
                        y1={toGraphY(value)}
                        x2={graphWidth - graphPadding}
                        y2={toGraphY(value)}
                        className="pose-graph-grid"
                      />
                    ))}

                    {axisXVisible && (
                      <line
                        x1={graphPadding}
                        y1={toGraphY(0)}
                        x2={graphWidth - graphPadding}
                        y2={toGraphY(0)}
                        className="pose-graph-axis"
                      />
                    )}
                    {axisYVisible && (
                      <line
                        x1={toGraphX(0)}
                        y1={graphPadding}
                        x2={toGraphX(0)}
                        y2={graphHeight - graphPadding}
                        className="pose-graph-axis"
                      />
                    )}

                    {waypointPolylinePoints && (
                      <polyline points={waypointPolylinePoints} className="waypoint-path-line" />
                    )}

                    <circle cx={currentX} cy={currentY} r="6" className="pose-graph-point-current" />

                    {waypoints.map((waypoint, index) => (
                      <g key={waypoint.id}>
                        <circle
                          cx={toGraphX(waypoint.x)}
                          cy={toGraphY(waypoint.y)}
                          r={index === activeWaypointIndex ? "6" : "5"}
                          className={index === activeWaypointIndex ? "waypoint-point-active" : "waypoint-point"}
                        />
                        <text
                          x={toGraphX(waypoint.x) + 7}
                          y={toGraphY(waypoint.y) - 8}
                          className="waypoint-point-label"
                        >
                          {index + 1}
                        </text>
                      </g>
                    ))}
                  </svg>

                  <p className="pose-graph-interaction-hint">
                    {tr(
                      "左クリックでウェイポイント追加。現在のTarget Yaw値が各ウェイポイントのYawとして保存されます。",
                      "Left click to add a waypoint. The current Target Yaw value is stored for each waypoint."
                    )}
                  </p>
                </section>

                <section className="waypoint-list-card">
                  <div className="waypoint-summary-row">
                    <span>{tr("登録数", "Count")}: {waypoints.length}</span>
                    <span>{tr("アクティブ", "Active")}: {activeWaypointIndex >= 0 ? activeWaypointIndex + 1 : "-"}</span>
                  </div>

                  <div className="waypoint-list-box">
                    {waypoints.length === 0 && (
                      <p className="connection-hint">{tr("ウェイポイント未登録", "No waypoints")}</p>
                    )}

                    {waypoints.map((waypoint, index) => (
                      <div
                        key={waypoint.id}
                        className={`waypoint-row ${index === activeWaypointIndex ? "waypoint-row-active" : ""}`}
                      >
                        <div className="waypoint-row-text">
                          <strong>{waypoint.label}</strong>
                          <span>
                            X: {waypoint.x.toFixed(3)}, Y: {waypoint.y.toFixed(3)}, Yaw: {waypoint.yawDeg.toFixed(1)}
                          </span>
                        </div>
                        <div className="waypoint-row-actions">
                          <button
                            className="connection-button btn-send waypoint-row-button"
                            onClick={() => setWaypointAsActive(index, true)}
                            disabled={!operationArmed}
                          >
                            {tr("適用&送信", "Apply & Send")}
                          </button>
                          <button
                            className="connection-button btn-neutral waypoint-row-button"
                            onClick={() => setWaypointAsActive(index, false)}
                          >
                            {tr("適用", "Apply")}
                          </button>
                          <button
                            className="serial-clear-button waypoint-row-button"
                            onClick={() => removeWaypointById(waypoint.id)}
                          >
                            {tr("削除", "Delete")}
                          </button>
                        </div>
                      </div>
                    ))}
                  </div>
                </section>
              </div>

              <p className="connection-hint">{translateRuntimeText(waypointInfo)}</p>
            </section>
          )}

          {isPageActive("planner") && (
            <section className="serial-bridge-panel planner-panel">
              <h2 className="serial-packet-title">{tr("プランナー", "Task Planner")}</h2>
              <p className="serial-packet-hint">
                {tr(
                  "状態、コート色、MFFマス番号、遷移モードを監視し、手動遷移と自動遷移を切り替えます。",
                  "Monitor state, coat color, MFF cell, and transition mode. Switch between manual and auto transitions."
                )}
              </p>

              <div className="planner-top-grid">
                <div className="planner-field-card">
                  <div className="planner-field-header">
                    <h3 className="serial-bridge-title">{tr("MFF現在地", "MFF Position")}</h3>
                    <div className="planner-field-header-actions">
                      <span className="planner-field-badge">
                        {tr("選択中の色", "Selected color")}: {plannerColorLabel}
                      </span>
                      <button
                        type="button"
                        className={`planner-rotate-button ${plannerFieldRotationDeg !== 0 ? "planner-rotate-button-active" : ""}`}
                        onClick={() => setPlannerFieldRotationDeg((prev) => (prev + PLANNER_FIELD_ROTATION_STEP_DEG) % 360)}
                      >
                        {tr(`図回転: ${plannerFieldRotationDeg}°`, `Rotation: ${plannerFieldRotationDeg}°`)}
                      </button>
                    </div>
                  </div>
                  <p className="connection-hint planner-field-hint">
                    {tr(
                      "緑マスの番号に合わせて現在地を表示します。コートを選択すると該当側だけ表示し、未設定時は両方表示します。",
                      "The numbered green cells show the current position. Selecting a court shows only that side; unknown shows both sides."
                    )}
                  </p>

                  <div className="planner-height-legend">
                    <span className="planner-height-chip planner-height-mm-200">200mm</span>
                    <span className="planner-height-chip planner-height-mm-0">0mm</span>
                    <span className="planner-height-chip planner-height-mm-400">400mm</span>
                  </div>

                  <div
                    className={`planner-field-layout-grid ${isSingleCourtView ? "planner-field-layout-grid-single" : ""}`}
                    style={{ "--planner-field-rotation": `${plannerFieldRotationDeg}deg` }}
                  >
                    {showRedCourt && <section className="planner-field-court planner-field-court-red">
                      <div className="planner-field-court-title-row">
                        <h4>{tr("赤コート", "Red Court")}</h4>
                        <span>{tr("左側レイアウト", "Left-side layout")}</span>
                      </div>
                      <div className="planner-field-cells">
                        {plannerLayoutRed.flatMap((row) => row.map((cell) => {
                          const isCurrent = plannerColorCode === 1 && plannerCellCode === cell;
                          const heightLevel = getMffHeightLevel(cell);
                          return (
                            <button
                              key={`mff-red-${cell}`}
                              type="button"
                              className={`planner-field-cell planner-field-cell-red planner-field-height-${heightLevel} ${isCurrent ? "planner-field-cell-current" : ""}`}
                              onClick={() => {
                                setPlannerColorCode(1);
                                setPlannerCellInput(String(cell));
                                publishPlannerColor(1);
                                publishPlannerCellValue(cell);
                              }}
                              title={tr(`赤コート ${cell}番`, `Red court cell ${cell}`)}
                            >
                              <span className="planner-field-cell-number">{cell}</span>
                              {isCurrent && <span className="planner-field-current-tag">{tr("現在地", "Current")}</span>}
                            </button>
                          );
                        }))}
                      </div>
                    </section>}

                    {showBlueCourt && <section className="planner-field-court planner-field-court-blue">
                      <div className="planner-field-court-title-row">
                        <h4>{tr("青コート", "Blue Court")}</h4>
                        <span>{tr("右側レイアウト", "Right-side layout")}</span>
                      </div>
                      <div className="planner-field-cells">
                        {plannerLayoutBlue.flatMap((row) => row.map((cell) => {
                          const isCurrent = plannerColorCode === 0 && plannerCellCode === cell;
                          const heightLevel = getMffHeightLevel(cell);
                          return (
                            <button
                              key={`mff-blue-${cell}`}
                              type="button"
                              className={`planner-field-cell planner-field-cell-blue planner-field-height-${heightLevel} ${isCurrent ? "planner-field-cell-current" : ""}`}
                              onClick={() => {
                                setPlannerColorCode(0);
                                setPlannerCellInput(String(cell));
                                publishPlannerColor(0);
                                publishPlannerCellValue(cell);
                              }}
                              title={tr(`青コート ${cell}番`, `Blue court cell ${cell}`)}
                            >
                              <span className="planner-field-cell-number">{cell}</span>
                              {isCurrent && <span className="planner-field-current-tag">{tr("現在地", "Current")}</span>}
                            </button>
                          );
                        }))}
                      </div>
                    </section>}
                  </div>
                </div>

                <section className="serial-bridge-card planner-status-card planner-status-card-top">
                  <h3 className="serial-bridge-title">{tr("状態監視", "Status Monitor")}</h3>
                  <div className="pose-current-grid planner-current-grid">
                    <div className="pose-current-item">
                      <span>{tr("状態", "State")}</span>
                      <strong>{plannerStateLabel}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("色", "Color")}</span>
                      <strong>{plannerColorLabel}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("MFFマス", "MFF Cell")}</span>
                      <strong>{plannerCellCode === 0 ? tr("未設定", "Unset") : plannerCellCode}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("遷移", "Transition")}</span>
                      <strong>{plannerModeLabel}</strong>
                    </div>
                  </div>
                  <p className="connection-hint planner-status-text">{plannerStatusText || tr("状態テキスト未受信", "No status text yet")}</p>
                </section>
              </div>

              <section className="serial-bridge-card planner-controls-panel">
                <div className="planner-toolbar planner-toolbar-bottom">
                  <div className="planner-toolbar-main-controls">
                    <button
                      className={`toggle-button ${plannerTransitionModeCode === 1 ? "toggle-on" : "toggle-off"}`}
                      onClick={togglePlannerTransitionMode}
                    >
                      {tr("遷移モード切替", "Toggle Transition Mode")}: {plannerModeLabel}
                    </button>
                    <button
                      className={`toggle-button ${plannerAutoSendEnabled ? "toggle-on" : "toggle-off"}`}
                      onClick={() => publishPlannerAutoSendEnabled(!plannerAutoSendEnabled)}
                    >
                      {plannerAutoSendEnabled
                        ? tr("遷移時自動送信: ON", "Auto Send on Transition: ON")
                        : tr("遷移時自動送信: OFF", "Auto Send on Transition: OFF")}
                    </button>
                    <div className="planner-toolbar-color-cell">
                      <div className="planner-color-buttons planner-color-buttons-inline">
                        <button
                          className={`connection-button planner-choice-button planner-color-blue ${plannerColorCode === 0 ? "planner-choice-selected" : ""}`}
                          onClick={() => publishPlannerColor(0)}
                        >
                          {tr("青", "Blue")}
                        </button>
                        <button
                          className={`connection-button planner-choice-button planner-color-red ${plannerColorCode === 1 ? "planner-choice-selected" : ""}`}
                          onClick={() => publishPlannerColor(1)}
                        >
                          {tr("赤", "Red")}
                        </button>
                        <button
                          className={`connection-button planner-choice-button planner-color-unknown ${plannerColorCode === -1 ? "planner-choice-selected" : ""}`}
                          onClick={() => publishPlannerColor(-1)}
                        >
                          {tr("未", "Unset")}
                        </button>
                      </div>

                      <div className="planner-cell-row planner-cell-row-inline">
                        <label className="serial-packet-label planner-cell-label planner-cell-label-inline">
                          {tr("マス", "Cell")}
                          <input
                            className="connection-input"
                            type="number"
                            min="0"
                            max="12"
                            value={plannerCellInput}
                            onChange={(e) => setPlannerCellInput(e.target.value)}
                          />
                        </label>
                        <button className="connection-button btn-send" onClick={publishPlannerCell}>
                          {tr("更新", "Apply")}
                        </button>
                      </div>
                    </div>
                  </div>
                  <span className="connection-hint planner-toolbar-mode-hint">
                    {tr("現在のモード", "Current Mode")}: {plannerModeLabel}
                  </span>
                </div>

                <section className="planner-status-card planner-manual-transition-card">
                  <h3 className="serial-bridge-title">{tr("手動遷移", "Manual Transition")}</h3>
                  <p className="connection-hint">
                    {tr("手動モードでのみ有効です。状態を直接選択します。", "Available only in manual mode. Select the state directly.")}
                  </p>
                  <div className="planner-state-buttons">
                    {plannerStateSequence.map((stateCode, index) => {
                      const label = getPlannerStateLabel(stateCode, language, plannerCustomStateLabelMap);
                      const isSelected = plannerStateCode === stateCode;
                      return (
                        <button
                          key={`planner-state-${stateCode}`}
                          className={`connection-button planner-choice-button planner-state-choice ${isSelected ? "planner-choice-selected" : ""} ${isSelected ? "btn-send" : "btn-connect"}`}
                          onClick={() => publishPlannerState(stateCode)}
                          disabled={plannerTransitionModeCode === 1}
                        >
                          {index + 1}. {label}
                        </button>
                      );
                    })}
                  </div>
                </section>

                <section className="planner-state-config-panel">
                  <div className="planner-state-config-header">
                    <h3 className="serial-bridge-title">{tr("状態設定", "State Configuration")}</h3>
                    <div className="planner-state-config-actions">
                      <button className="connection-button btn-connect" onClick={() => publishPlannerStateSequence()}>
                        {tr("順序を送信", "Send Sequence")}
                      </button>
                      <button className="serial-clear-button" onClick={resetPlannerStateSequence}>
                        {tr("順序を初期化", "Reset Sequence")}
                      </button>
                      <button className="connection-button btn-save" onClick={exportPlannerStateConfig}>
                        {tr("設定をエクスポート", "Export Config")}
                      </button>
                      <button
                        className="connection-button btn-restore"
                        onClick={() => plannerConfigImportInputRef.current?.click()}
                      >
                        {tr("設定をインポート", "Import Config")}
                      </button>
                      <input
                        ref={plannerConfigImportInputRef}
                        type="file"
                        accept=".json,application/json"
                        style={{ display: "none" }}
                        onChange={importPlannerStateConfigFromFile}
                      />
                    </div>
                  </div>
                  <p className="connection-hint planner-state-config-hint">
                    {tr(
                      "順序、状態ごとの目標座標、状態ごとのドライブモード、オドメトリリセット有無をここでまとめて設定します。未指定の座標やモードは送信しません。",
                      "Configure state order, per-state target pose, per-state drive mode, and per-state odometry reset here. Unset pose or mode values will not be sent."
                    )}
                  </p>

                  <section className="planner-state-config-subcard" style={{ marginBottom: 10 }}>
                    <div className="planner-state-config-subheader">
                      <h5>{tr("保存済み設定の読み込み", "Load Saved Configuration")}</h5>
                    </div>
                    <button className="connection-button btn-connect" onClick={() => loadPlannerExportsList()} style={{ marginBottom: 8 }}>
                      {tr("一覧を更新", "Refresh List")}
                    </button>
                    {plannerExportsList.length === 0 ? (
                      <span className="connection-hint">{tr("保存済み設定はありません", "No saved configurations")}</span>
                    ) : (
                      <div style={{ maxHeight: "150px", overflowY: "auto", border: "1px solid #ddd", borderRadius: "4px", padding: "4px" }}>
                        {plannerExportsList.map((file) => (
                          <div
                            key={`planner-export-${file.fileName}`}
                            style={{
                              padding: "4px 6px",
                              marginBottom: "2px",
                              backgroundColor: plannerSelectedExportPath === file.relativePath ? "#e0f7ff" : "#f5f5f5",
                              border: "1px solid #ccc",
                              borderRadius: "3px",
                              cursor: "pointer",
                              fontSize: "0.9em",
                            }}
                            onClick={() => setPlannerSelectedExportPath(file.relativePath)}
                          >
                            <div style={{ fontWeight: "bold" }}>{file.fileName}</div>
                            <div style={{ fontSize: "0.85em", color: "#666" }}>
                              {new Date(file.timestamp).toLocaleString()} ({(file.size / 1024).toFixed(1)} KB)
                            </div>
                          </div>
                        ))}
                      </div>
                    )}
                    {plannerSelectedExportPath && (
                      <button
                        className="connection-button btn-connect"
                        onClick={() => loadPlannerExportFile(plannerSelectedExportPath)}
                        style={{ marginTop: 8, width: "100%" }}
                      >
                        {tr("選択したファイルを読み込み", "Load Selected File")}
                      </button>
                    )}
                  </section>

                  <section className="planner-state-config-subcard" style={{ marginBottom: 10 }}>
                    <div className="planner-state-config-subheader">
                      <h5>{tr("状態の追加/削除", "Add/Remove States")}</h5>
                    </div>
                    <div className="planner-state-config-fields">
                      <label className="serial-packet-label">
                        {tr("コード", "Code")}
                        <input
                          className="connection-input"
                          type="number"
                          min="6"
                          step="1"
                          value={plannerNewStateCodeInput}
                          onChange={(e) => setPlannerNewStateCodeInput(e.target.value)}
                          placeholder="6+"
                        />
                      </label>
                      <label className="serial-packet-label">
                        {tr("表示名(日本語)", "Label (JA)")}
                        <input
                          className="connection-input"
                          value={plannerNewStateJaInput}
                          onChange={(e) => setPlannerNewStateJaInput(e.target.value)}
                          placeholder={tr("例: 状態A", "e.g. Stage A")}
                        />
                      </label>
                      <label className="serial-packet-label">
                        {tr("表示名(英語)", "Label (EN)")}
                        <input
                          className="connection-input"
                          value={plannerNewStateEnInput}
                          onChange={(e) => setPlannerNewStateEnInput(e.target.value)}
                          placeholder="e.g. State A"
                        />
                      </label>
                    </div>
                    <button className="connection-button btn-connect planner-state-config-apply" onClick={addPlannerCustomState}>
                      {tr("状態を追加", "Add State")}
                    </button>
                    <div className="planner-state-mode-buttons" style={{ marginTop: 8 }}>
                      {plannerCustomStates.length === 0 && (
                        <span className="connection-hint">{tr("追加状態はありません", "No custom states")}</span>
                      )}
                      {plannerCustomStates.map((state) => (
                        <button
                          key={`planner-custom-state-remove-${state.code}`}
                          className="serial-clear-button"
                          onClick={() => removePlannerCustomState(state.code)}
                          title={`${state.code}: ${state.labelJa} / ${state.labelEn}`}
                        >
                          {tr("削除", "Remove")} {state.code}
                        </button>
                      ))}
                    </div>
                  </section>

                  <div className="planner-state-config-list">
                    {plannerStateSequence.map((stateCode, index) => {
                      const poseConfig = plannerStatePoseConfig[stateCode] || { enabled: false, x: "0.0", y: "0.0", yaw: "0.0" };
                      const modeConfig = plannerStateModeConfig[stateCode] || { enabled: false, modeCode: 1 };
                      const odomResetConfig = plannerStateOdomResetConfig[stateCode] || { enabled: false };
                      const stateLabel = getPlannerStateLabel(stateCode, language, plannerCustomStateLabelMap);

                      return (
                        <article key={`planner-state-config-${stateCode}`} className="planner-state-config-card">
                          <div className="planner-state-config-card-header">
                            <div className="planner-state-config-card-title">
                              <span className="planner-state-config-index">#{index + 1}</span>
                              <h4>{stateLabel}</h4>
                            </div>
                            <div className="planner-state-config-reorder">
                              <button
                                className="connection-button btn-neutral"
                                onClick={() => movePlannerStateSequence(stateCode, -1)}
                                disabled={index === 0}
                              >
                                {tr("上へ", "Up")}
                              </button>
                              <button
                                className="connection-button btn-neutral"
                                onClick={() => movePlannerStateSequence(stateCode, 1)}
                                disabled={index === plannerStateSequence.length - 1}
                              >
                                {tr("下へ", "Down")}
                              </button>
                            </div>
                          </div>

                          <div className="planner-state-config-grid">
                            <section className="planner-state-config-subcard">
                              <div className="planner-state-config-subheader">
                                <h5>{tr("座標・姿勢", "Pose")}</h5>
                                <button
                                  className={`toggle-button ${poseConfig.enabled ? "toggle-on" : "toggle-off"}`}
                                  onClick={() => updatePlannerStatePose(stateCode, "enabled", !poseConfig.enabled)}
                                >
                                  {poseConfig.enabled ? tr("指定中", "Enabled") : tr("未指定", "Unset")}
                                </button>
                              </div>
                              <div className="planner-state-config-fields">
                                <label className="serial-packet-label">
                                  X
                                  <input
                                    className="connection-input"
                                    type="number"
                                    step="0.01"
                                    value={poseConfig.x}
                                    onChange={(e) => updatePlannerStatePose(stateCode, "x", e.target.value)}
                                  />
                                </label>
                                <label className="serial-packet-label">
                                  Y
                                  <input
                                    className="connection-input"
                                    type="number"
                                    step="0.01"
                                    value={poseConfig.y}
                                    onChange={(e) => updatePlannerStatePose(stateCode, "y", e.target.value)}
                                  />
                                </label>
                                <label className="serial-packet-label">
                                  Yaw [rad]
                                  <input
                                    className="connection-input"
                                    type="number"
                                    step="0.01"
                                    value={poseConfig.yaw}
                                    onChange={(e) => updatePlannerStatePose(stateCode, "yaw", e.target.value)}
                                  />
                                </label>
                              </div>
                              <button className="connection-button btn-send planner-state-config-apply" onClick={() => publishPlannerStatePose(stateCode)}>
                                {tr("座標を送信", "Send Pose")}
                              </button>
                            </section>

                            <section className="planner-state-config-subcard">
                              <div className="planner-state-config-subheader">
                                <h5>{tr("モード", "Mode")}</h5>
                                <button
                                  className={`toggle-button ${modeConfig.enabled ? "toggle-on" : "toggle-off"}`}
                                  onClick={() => updatePlannerStateMode(stateCode, modeConfig.enabled ? -1 : modeConfig.modeCode)}
                                >
                                  {modeConfig.enabled ? tr("指定中", "Enabled") : tr("未指定", "Unset")}
                                </button>
                              </div>
                              <div className="planner-state-mode-buttons">
                                {Object.entries(DRIVE_MODE_LABELS).map(([modeCode, label]) => {
                                  const numericModeCode = Number(modeCode);
                                  const isSelected = modeConfig.enabled && modeConfig.modeCode === numericModeCode;
                                  return (
                                    <button
                                      key={`planner-state-mode-${stateCode}-${modeCode}`}
                                      className={`connection-button planner-choice-button ${isSelected ? "planner-choice-selected" : ""}`}
                                      onClick={() => updatePlannerStateMode(stateCode, numericModeCode)}
                                    >
                                      {getDriveModeLabel(numericModeCode, language)}
                                    </button>
                                  );
                                })}
                              </div>
                              <button className="connection-button btn-send planner-state-config-apply" onClick={() => publishPlannerStateMode(stateCode)}>
                                {tr("モードを送信", "Send Mode")}
                              </button>
                            </section>

                            <section className="planner-state-config-subcard">
                              <div className="planner-state-config-subheader">
                                <h5>{tr("オドメトリリセット", "Odometry Reset")}</h5>
                                <button
                                  className={`toggle-button ${odomResetConfig.enabled ? "toggle-on" : "toggle-off"}`}
                                  onClick={() => updatePlannerStateOdomReset(stateCode, !odomResetConfig.enabled)}
                                >
                                  {odomResetConfig.enabled ? tr("送信する", "Send") : tr("送信しない", "Off")}
                                </button>
                              </div>
                              <p className="connection-hint">
                                {tr(
                                  "この状態へ遷移したときに odom_reset を1回送信します。",
                                  "Send odom_reset once when transitioning into this state."
                                )}
                              </p>
                              <button className="connection-button btn-send planner-state-config-apply" onClick={() => publishPlannerStateOdomReset(stateCode)}>
                                {tr("リセット設定を送信", "Send Reset Setting")}
                              </button>
                            </section>
                          </div>
                        </article>
                      );
                    })}
                  </div>
                </section>

              </section>
            </section>
          )}

          {isPageActive("teaching") && (
            <section className="teaching-panel">
              <h2 className="serial-packet-title">{tr("ティーチング", "Teaching / Trace Replay")}</h2>
              <p className="serial-packet-hint">
                {tr(
                  "手動操作で機体を動かしながら現在姿勢を記録し、あとで同じ順序で目標値として再生します。",
                  "Record current poses while manually driving, then replay them as sequential target values."
                )}
              </p>

              <div className="teaching-toolbar">
                <button className="connection-button btn-connect" onClick={startTraceRecording} disabled={traceRecording}>
                  {tr("記録開始", "Start Rec")}
                </button>
                <button className="connection-button btn-neutral" onClick={stopTraceRecording} disabled={!traceRecording}>
                  {tr("記録停止", "Stop Rec")}
                </button>
                <button className="connection-button btn-send" onClick={appendCurrentPoseToTrace}>
                  {tr("現在姿勢を1点記録", "Record Current Pose")}
                </button>
                <button className="connection-button btn-connect" onClick={startTraceReplay} disabled={traceReplayRunning || tracePoints.length === 0}>
                  {tr("再生開始", "Start Replay")}
                </button>
                <button className="connection-button btn-neutral" onClick={stopTraceReplay} disabled={!traceReplayRunning}>
                  {tr("再生停止", "Stop Replay")}
                </button>
                <button className="serial-clear-button" onClick={clearTracePoints}>
                  {tr("記録全削除", "Clear All")}
                </button>
                <button className="connection-button btn-save" onClick={exportTracePoints}>
                  {tr("エクスポート", "Export")}
                </button>
                <button
                  className="connection-button btn-restore"
                  onClick={() => traceImportInputRef.current?.click()}
                >
                  {tr("インポート", "Import")}
                </button>
                <input
                  ref={traceImportInputRef}
                  type="file"
                  accept=".json,application/json"
                  style={{ display: "none" }}
                  onChange={importTracePointsFromFile}
                />
              </div>

              <div className="teaching-settings-grid">
                <label className="serial-packet-label">
                  {tr("記録周期 [ms]", "Record Interval [ms]")}
                  <input
                    className="connection-input"
                    type="number"
                    min="50"
                    max="10000"
                    step="10"
                    value={traceSampleMsInput}
                    onChange={(e) => setTraceSampleMsInput(e.target.value)}
                  />
                </label>

                <label className="serial-packet-label">
                  {tr("再生周期 [ms]", "Replay Interval [ms]")}
                  <input
                    className="connection-input"
                    type="number"
                    min="50"
                    max="10000"
                    step="10"
                    value={traceReplayMsInput}
                    onChange={(e) => setTraceReplayMsInput(e.target.value)}
                  />
                </label>

                <button
                  className={`toggle-button ${traceReplayLoop ? "toggle-on" : "toggle-off"}`}
                  onClick={() => setTraceReplayLoop((prev) => !prev)}
                >
                  {traceReplayLoop ? tr("再生ループ: ON", "Replay Loop: ON") : tr("再生ループ: OFF", "Replay Loop: OFF")}
                </button>

                <button
                  className={`toggle-button ${traceReplayAutoPublish ? "toggle-on" : "toggle-off"}`}
                  onClick={() => setTraceReplayAutoPublish((prev) => !prev)}
                >
                  {traceReplayAutoPublish ? tr("再生時送信: ON", "Publish on Replay: ON") : tr("再生時送信: OFF", "Publish on Replay: OFF")}
                </button>
              </div>

              <div className="teaching-status-row">
                <span>{tr("記録", "Recording")}: {traceRecording ? "ON" : "OFF"}</span>
                <span>{tr("再生", "Replay")}: {traceReplayRunning ? "ON" : "OFF"}</span>
                <span>{tr("点数", "Points")}: {tracePoints.length}</span>
                <span>{tr("再生インデックス", "Replay Index")}: {traceReplayIndex >= 0 ? traceReplayIndex + 1 : "-"}</span>
              </div>

              <div className="teaching-overview-grid">
                <section className="pose-graph-card">
                  <div className="pose-graph-title-row">
                    <h3 className="pose-graph-title">{tr("記録点グラフ", "Trace Points Graph")}</h3>
                    <span className="pose-graph-scale">{tr("単位: m", "Unit: m")}</span>
                  </div>

                  <svg
                    className="pose-graph teaching-graph"
                    viewBox={`0 0 ${graphWidth} ${graphHeight}`}
                    role="img"
                    aria-label={tr("ティーチング記録点グラフ", "Teaching trace points graph")}
                  >
                    <rect
                      x={graphPadding}
                      y={graphPadding}
                      width={graphInnerWidth}
                      height={graphInnerHeight}
                      className="pose-graph-frame"
                    />

                    {traceGridXValues.map((value) => (
                      <line
                        key={`trace-grid-x-${value}`}
                        x1={toTraceGraphX(value)}
                        y1={graphPadding}
                        x2={toTraceGraphX(value)}
                        y2={graphHeight - graphPadding}
                        className="pose-graph-grid"
                      />
                    ))}
                    {traceGridYValues.map((value) => (
                      <line
                        key={`trace-grid-y-${value}`}
                        x1={graphPadding}
                        y1={toTraceGraphY(value)}
                        x2={graphWidth - graphPadding}
                        y2={toTraceGraphY(value)}
                        className="pose-graph-grid"
                      />
                    ))}

                    {traceAxisXVisible && (
                      <line
                        x1={graphPadding}
                        y1={toTraceGraphY(0)}
                        x2={graphWidth - graphPadding}
                        y2={toTraceGraphY(0)}
                        className="pose-graph-axis"
                      />
                    )}
                    {traceAxisYVisible && (
                      <line
                        x1={toTraceGraphX(0)}
                        y1={graphPadding}
                        x2={toTraceGraphX(0)}
                        y2={graphHeight - graphPadding}
                        className="pose-graph-axis"
                      />
                    )}

                    {tracePoints.length > 1 && (
                      <polyline
                        points={tracePoints
                          .map((point) => `${toTraceGraphX(point.x)},${toTraceGraphY(point.y)}`)
                          .join(" ")}
                        className="trace-point-path-line"
                      />
                    )}

                    {tracePoints.map((point, index) => {
                      const pointX = toTraceGraphX(point.x);
                      const pointY = toTraceGraphY(point.y);
                      const pointYaw = normalizeYawRad(point.yawRad + yawOffsetRad);
                      const arrowLength = 15;
                      const arrowEndX = pointX + Math.cos(pointYaw) * arrowLength;
                      const arrowEndY = pointY - Math.sin(pointYaw) * arrowLength;
                      return (
                        <line
                          key={`trace-arrow-${point.id}`}
                          x1={pointX}
                          y1={pointY}
                          x2={arrowEndX}
                          y2={arrowEndY}
                          className={index === traceReplayIndex ? "pose-graph-arrow-current" : "pose-graph-arrow-target"}
                          style={{ opacity: index === traceReplayIndex ? 1 : 0.6 }}
                        />
                      );
                    })}

                    <circle cx={toTraceGraphX(poseX)} cy={toTraceGraphY(poseY)} r="6" className="pose-graph-point-current" />

                    {tracePoints.map((point, index) => (
                      <g key={`trace-${point.id}`}>
                        <circle
                          cx={toTraceGraphX(point.x)}
                          cy={toTraceGraphY(point.y)}
                          r={index === traceReplayIndex ? "6" : "5"}
                          className={index === traceReplayIndex ? "trace-point-active" : "trace-point"}
                        />
                        <text
                          x={toTraceGraphX(point.x) + 7}
                          y={toTraceGraphY(point.y) - 8}
                          className={index === traceReplayIndex ? "trace-point-label trace-point-label-active" : "trace-point-label"}
                        >
                          {index + 1}
                        </text>
                      </g>
                    ))}

                    <text x={graphPadding} y={graphPadding - 8} className="pose-graph-corner-label">
                      Y {traceGraphMaxY.toFixed(2)}
                    </text>
                    <text x={graphPadding} y={graphHeight - 8} className="pose-graph-corner-label">
                      Y {traceGraphMinY.toFixed(2)}
                    </text>
                    <text x={graphPadding} y={graphHeight - graphPadding + 16} className="pose-graph-corner-label">
                      X {traceGraphMinX.toFixed(2)}
                    </text>
                    <text x={graphWidth - graphPadding - 84} y={graphHeight - graphPadding + 16} className="pose-graph-corner-label">
                      X {traceGraphMaxX.toFixed(2)}
                    </text>
                  </svg>

                  <div className="pose-graph-legend">
                    <span className="pose-legend-item">
                      <i className="pose-legend-dot pose-legend-current" />
                      {tr("現在位置", "Current position")}
                    </span>
                    <span className="pose-legend-item">
                      <i className="pose-legend-dot pose-legend-target" />
                      {tr("記録点", "Trace points")}
                    </span>
                  </div>
                </section>

                <section className="trace-list-card">
                  <div className="waypoint-summary-row">
                    <span>{tr("登録数", "Count")}: {tracePoints.length}</span>
                    <span>{tr("再生中", "Playing")}: {traceReplayIndex >= 0 ? traceReplayIndex + 1 : "-"}</span>
                  </div>

                  <div className="teaching-list-box">
                    {tracePoints.length === 0 && (
                      <p className="connection-hint">{tr("記録データはまだありません", "No trace points recorded yet")}</p>
                    )}

                    {tracePoints.map((point, index) => (
                      <div
                        key={point.id}
                        className={`teaching-row ${index === traceReplayIndex ? "teaching-row-active" : ""}`}
                      >
                        <div className="teaching-row-text">
                          <strong>{point.label}</strong>
                          <span>
                            X: {point.x.toFixed(3)}, Y: {point.y.toFixed(3)}, Yaw: {point.yawDeg.toFixed(1)} ({point.at})
                          </span>
                        </div>
                        <div className="teaching-row-actions">
                          <button
                            className="connection-button btn-neutral teaching-row-button"
                            onClick={() => applyTracePointToTarget(index, false)}
                          >
                            {tr("適用", "Apply")}
                          </button>
                          <button
                            className="connection-button btn-send teaching-row-button"
                            onClick={() => applyTracePointToTarget(index, true)}
                            disabled={!operationArmed}
                          >
                            {tr("適用&送信", "Apply & Send")}
                          </button>
                          <button
                            className="serial-clear-button teaching-row-button"
                            onClick={() => removeTracePointById(point.id)}
                          >
                            {tr("削除", "Delete")}
                          </button>
                        </div>
                      </div>
                    ))}
                  </div>
                </section>
              </div>

              <p className="connection-hint">{translateRuntimeText(traceInfo)}</p>
            </section>
          )}

          {isPageActive("simulator") && (
            <section className="serial-bridge-panel">
              <h2 className="serial-packet-title">{tr("仮想オドメトリ", "Virtual Odometry")}</h2>
              <p className="serial-packet-hint">
                {tr("仮想オドメトリを発行します。", "Publish virtual odometry data.")}
              </p>
              {renderVirtualOdomPanel("simulator")}
            </section>
          )}

          {isPageActive("actuator") && (
            <section className="serial-packet-section">
              <h2 className="serial-packet-title">{tr("ダイレクト送信", "Direct Transmission")} (Int16MultiArray)</h2>
              <p className="serial-packet-hint">
                {tr("IDで送信先を切替えることができます。トピック名:", "Switch destination by ID. Topic:")} <strong>{serialTopicName}</strong>
                <br />
                {tr("入力値のチェックは行っていません。注意して入力してください。", "Input values are not validated. Use carefully.")}
              </p>

              <div className="serial-packet-controls">
                <label className="serial-packet-label">
                  ID
                  <input
                    className="connection-input"
                    type="number"
                    min="0"
                    max="255"
                    value={serialTargetIdInput}
                    onChange={(e) => setSerialTargetIdInput(e.target.value)}
                  />
                </label>
                <label className="serial-packet-label">
                  {tr("要素数", "Length")}
                  <input
                    className="connection-input"
                    type="number"
                    min="1"
                    max="64"
                    value={serialElementCount}
                    onChange={(e) => updateSerialElementCount(e.target.value)}
                  />
                </label>
                <label className="serial-packet-label">
                  DEBUG
                  <input
                    className="connection-input"
                    type="number"
                    min="0"
                    max="1"
                    value={serialValues[0] ?? 0}
                    onChange={(e) => updateSerialValue(0, e.target.value)}
                  />
                </label>
              </div>

              <div className="actuator-groups">
                <section className="actuator-group">
                  <h3 className="actuator-group-title">{tr("モータ", "Motors")} (MD1 - MD8)</h3>
                  <div className="serial-packet-grid">
                    {Array.from({ length: 8 }, (_, i) => i + 1)
                      .filter((index) => index < serialElementCount)
                      .map((index) => renderSerialInputItem(index))}
                  </div>
                </section>

                <section className="actuator-group">
                  <h3 className="actuator-group-title">{tr("サーボ", "Servos")} (SERVO1 - SERVO8)</h3>
                  <div className="serial-packet-grid">
                    {Array.from({ length: 8 }, (_, i) => i + 9)
                      .filter((index) => index < serialElementCount)
                      .map((index) => renderSerialInputItem(index))}
                  </div>
                </section>

                <section className="actuator-group">
                  <h3 className="actuator-group-title">{tr("TR出力", "TR Outputs")} (TR1 - TR7)</h3>
                  <div className="serial-packet-grid">
                    {Array.from({ length: 7 }, (_, i) => i + 17)
                      .filter((index) => index < serialElementCount)
                      .map((index) => renderSerialInputItem(index))}
                  </div>
                </section>

                <section className="actuator-group actuator-actions-group">
                  <h3 className="actuator-group-title">{tr("配列操作", "Array Operations")}</h3>
                  <div className="serial-packet-actions">
                    <button className="connection-button serial-send-button btn-send" onClick={() => publishSerialPacket(true)} disabled={!operationArmed}>
                      {tr("配列送信", "Send Array")}
                    </button>
                    <button className="serial-clear-button" onClick={clearSerialPacket}>
                      {tr("配列クリア", "Clear Array")}
                    </button>

                    <label className="serial-packet-label">
                      {tr("定期送信 Hz", "Periodic Hz")}
                      <select
                        className="connection-input"
                        value={serialPeriodicHz}
                        onChange={(e) => setSerialPeriodicHz(e.target.value)}
                      >
                        <option value="1">1</option>
                        <option value="2">2</option>
                        <option value="5">5</option>
                        <option value="10">10</option>
                        <option value="20">20</option>
                        <option value="30">30</option>
                        <option value="50">50</option>
                      </select>
                    </label>

                    <button
                      className={`serial-periodic-button ${serialPeriodicEnabled ? "serial-periodic-on" : ""}`}
                      disabled={!operationArmed}
                      onClick={() => setSerialPeriodicEnabled((prev) => !prev)}
                    >
                      {serialPeriodicEnabled ? tr("定期送信: ON", "Periodic: ON") : tr("定期送信: OFF", "Periodic: OFF")}
                    </button>
                  </div>
                </section>

                {serialElementCount > 24 && (
                  <section className="actuator-group">
                    <h3 className="actuator-group-title">{tr("追加チャネル", "Extra Channels")} (CH24+)</h3>
                    <div className="serial-packet-grid">
                      {Array.from({ length: serialElementCount - 24 }, (_, i) => i + 24)
                        .map((index) => renderSerialInputItem(index))}
                    </div>
                  </section>
                )}
              </div>

              <p className="connection-hint">{translateRuntimeText(serialPublishInfo)}</p>
            </section>
          )}

          {isPageActive("topic") && (
            <section className="topic-panel">
              <h2 className="serial-packet-title">{tr("ROS2 トピック監視", "ROS2 Topic Monitor")}</h2>
              <p className="serial-packet-hint">
                {tr("トピック一覧を取得し、選択したトピックを subscribe して内容を確認できます。", "Fetch topic list and subscribe selected topic to inspect messages.")}
              </p>

              <div className="topic-toolbar">
                <button className="connection-button btn-connect" onClick={refreshTopicList}>
                  {tr("トピック一覧を更新", "Refresh Topic List")}
                </button>
                <span className="connection-hint">
                  {topicListLoading ? tr("取得中...", "Loading...") : `${tr("件数", "Count")}: ${topicList.length}`}
                </span>
              </div>

              {topicListError && <p className="connection-hint topic-error">{translateRuntimeText(topicListError)}</p>}

              <div className="topic-select-row">
                <select
                  className="connection-input"
                  value={selectedEchoTopic}
                  onChange={(e) => {
                    const topicName = e.target.value;
                    setSelectedEchoTopic(topicName);
                    const matched = topicList.find((item) => item.name === topicName);
                    setSelectedEchoType(matched?.type || "");
                  }}
                >
                  <option value="">{tr("監視するトピックを選択", "Select topic to monitor")}</option>
                  {topicList.map((item) => (
                    <option key={item.name} value={item.name}>
                      {item.name} {item.type ? `(${item.type})` : ""}
                    </option>
                  ))}
                </select>

                <button className="connection-button btn-send" onClick={startTopicEcho}>
                  {tr("Echo開始", "Start Echo")}
                </button>
                <button className="serial-clear-button" onClick={stopTopicEcho}>
                  {tr("Echo停止", "Stop Echo")}
                </button>
              </div>

              <p className="connection-hint">
                {translateRuntimeText(topicEchoInfo)} {topicEchoRunning ? translateRuntimeText("(受信中)") : ""}
              </p>

              <div className="topic-list-box">
                {topicList.map((item) => (
                  <div key={item.name} className="topic-list-row">
                    <span className="topic-list-name">{item.name}</span>
                    <span className="topic-list-type">{item.type || translateRuntimeText("型不明")}</span>
                  </div>
                ))}
              </div>

              <h3 className="topic-echo-title">{tr("Echoログ (最新30件)", "Echo Log (latest 30)")}</h3>
              <div className="topic-echo-box">
                {topicEchoMessages.length === 0 && (
                  <p className="connection-hint">{translateRuntimeText("まだ受信していません")}</p>
                )}
                {topicEchoMessages.map((row) => (
                  <article key={row.id} className="topic-echo-item">
                    <header className="topic-echo-meta">{row.at}</header>
                    <pre className="topic-echo-pre">{row.payload}</pre>
                  </article>
                ))}
              </div>
            </section>
          )}

          {isPageActive("camera") && (
            <section className="topic-panel">
              <h2 className="serial-packet-title">{tr("Realsense 映像ストリーミング", "Realsense Camera Streaming")}</h2>
              <p className="serial-packet-hint">
                {tr("sensor_msgs/msg/Image を購読して映像を表示します。depth(16UC1) は擬似グレースケール表示します。", "Subscribe to sensor_msgs/msg/Image and render frames. depth (16UC1) is shown as pseudo grayscale.")}
              </p>

              <div className="topic-select-row">
                <input
                  className="connection-input"
                  value={cameraTopicInput}
                  onChange={(e) => setCameraTopicInput(e.target.value)}
                  placeholder="/camera/image_raw"
                />
                <button className="connection-button btn-send" onClick={startCameraStream}>
                  {tr("開始", "Start")}
                </button>
                <button className="serial-clear-button" onClick={stopCameraStream}>
                  {tr("停止", "Stop")}
                </button>
              </div>

              <div className="topic-select-row camera-preset-row">
                <button
                  className="connection-button btn-connect"
                  onClick={() => applyCameraTopicPreset("/camera/image_raw")}
                >
                  {tr("Webcam", "Webcam")}
                </button>
                <button
                  className="connection-button btn-connect"
                  onClick={() => applyCameraTopicPreset("/camera/camera/color/image_raw")}
                >
                  {tr("RealSense Color", "RealSense Color")}
                </button>
                <button
                  className="connection-button btn-connect"
                  onClick={() => applyCameraTopicPreset("/camera/camera/depth/image_rect_raw")}
                >
                  {tr("RealSense Depth", "RealSense Depth")}
                </button>
              </div>

              <p className="connection-hint">
                {translateRuntimeText(cameraStreamInfo)}
                {cameraStreamRunning ? ` | topic: ${cameraTopicName}` : ""}
              </p>

              <div className="serial-bridge-card serial-bridge-card-log" style={{ minHeight: 320 }}>
                {cameraFrameUrl ? (
                  <>
                    <img
                      src={cameraFrameUrl}
                      alt="camera stream"
                      style={{ width: "100%", maxHeight: 520, objectFit: "contain", borderRadius: 10 }}
                    />
                    <p className="connection-hint" style={{ marginTop: 8 }}>
                      {cameraFrameMeta.width} x {cameraFrameMeta.height} | {cameraFrameMeta.encoding || "unknown"} | {cameraFrameMeta.fps} fps
                    </p>
                  </>
                ) : (
                  <p className="connection-hint">{tr("映像未受信です。開始を押してください。", "No frame received yet. Press Start.")}</p>
                )}
              </div>
            </section>
          )}

          {isPageActive("cube-detection") && (
            <section className="topic-panel">
              <h2 className="serial-packet-title">{tr("立方体検知モニタ", "Cube Detection Monitor")}</h2>
              <p className="serial-packet-hint">
                {tr(
                  "深度画像の中央近傍から立方体候補を検出し、検出状態とデバッグ画像を表示します。",
                  "Detects a cube candidate near image center from depth image and displays status with debug frames."
                )}
              </p>

              <div className="topic-select-row">
                <input
                  className="connection-input"
                  value={cubeDebugTopicInput}
                  onChange={(e) => setCubeDebugTopicInput(e.target.value)}
                  onKeyDown={(e) => {
                    if (e.key === "Enter") applyCubeDebugTopicName();
                  }}
                  placeholder="/cube_detection/debug_image"
                />
                <button className="connection-button btn-connect" onClick={applyCubeDebugTopicName}>
                  {tr("更新", "Apply")}
                </button>
                <button className="connection-button btn-send" onClick={startCubeDebugStream}>
                  {tr("開始", "Start")}
                </button>
                <button className="serial-clear-button" onClick={stopCubeDebugStream}>
                  {tr("停止", "Stop")}
                </button>
              </div>

              <p className="connection-hint">
                {translateRuntimeText(cubeDebugStreamInfo)}
                {cubeDebugStreamRunning ? ` | topic: ${cubeDebugTopicName}` : ""}
              </p>

              <div className="pose-overview-grid">
                <section className="pose-graph-card" style={{ marginTop: 0 }}>
                  <div className="pose-graph-title-row">
                    <h3 className="pose-graph-title">{tr("デバッグ画像", "Debug Image")}</h3>
                    <span className="pose-graph-scale">{cubeDetected ? tr("検出中", "Detected") : tr("未検出", "Not Detected")}</span>
                  </div>

                  {cubeFrameUrl ? (
                    <>
                      <img
                        src={cubeFrameUrl}
                        alt="cube detection debug"
                        style={{ width: "100%", maxHeight: 520, objectFit: "contain", borderRadius: 10 }}
                      />
                      <p className="connection-hint" style={{ marginTop: 8 }}>
                        {cubeFrameMeta.width} x {cubeFrameMeta.height} | {cubeFrameMeta.encoding || "unknown"} | {cubeFrameMeta.fps} fps
                      </p>
                    </>
                  ) : (
                    <p className="connection-hint">{tr("デバッグ画像未受信です。開始を押してください。", "No debug frame received yet. Press Start.")}</p>
                  )}
                </section>

                <section className="pose-detail-panel">
                  <h3 className="pose-detail-title">{tr("検出ステータス", "Detection Status")}</h3>
                  <div className="pose-current-grid">
                    <div className="pose-current-item">
                      <span>{tr("検出", "Detected")}</span>
                      <strong>{cubeDetected ? tr("YES", "YES") : tr("NO", "NO")}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("中心X [norm]", "Center X [norm]")}</span>
                      <strong>{cubeInfo.cxNorm.toFixed(3)}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("中心Y [norm]", "Center Y [norm]")}</span>
                      <strong>{cubeInfo.cyNorm.toFixed(3)}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("幅 [norm]", "Width [norm]")}</span>
                      <strong>{cubeInfo.widthNorm.toFixed(3)}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("高さ [norm]", "Height [norm]")}</span>
                      <strong>{cubeInfo.heightNorm.toFixed(3)}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("深度 [m]", "Depth [m]")}</span>
                      <strong>{cubeInfo.depthM.toFixed(3)}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("信頼度", "Score")}</span>
                      <strong>{cubeInfo.score.toFixed(3)}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("面積 [px]", "Area [px]")}</span>
                      <strong>{cubeInfo.areaPx}</strong>
                    </div>
                    <div className="pose-current-item">
                      <span>{tr("更新時刻", "Updated")}</span>
                      <strong>{cubeUpdatedAt || "-"}</strong>
                    </div>
                  </div>
                </section>
              </div>
            </section>
          )}

          {isPageActive("serial-bridge") && (
            <section className="serial-bridge-panel">
              <h2 className="serial-packet-title">{tr("Serial Bridge 管理", "Serial Bridge")}</h2>
              <p className="serial-packet-hint">
                {tr("検出ポート、トピック由来のDevice ID、実行状態を確認し、ここから serial_bridge を手動起動できます。", "Check detected ports, topic-derived IDs, runtime status, and start serial_bridge manually.")}
                <br />
                {tr("serial_bridge はコンソール起動時には自動起動しません。", "serial_bridge does not auto-start with the console.")}
              </p>

              <div className="serial-bridge-toolbar">
                <button className="connection-button btn-connect" onClick={refreshSerialBridgeStatus}>
                  {tr("状態更新", "Refresh Status")}
                </button>
                <button className="connection-button btn-send" onClick={startSerialBridgeFromConsole}>
                  {tr("serial_bridge 起動", "Start serial_bridge")}
                </button>
                <button className="connection-button btn-neutral" onClick={stopSerialBridgeFromConsole}>
                  {tr("serial_bridge 停止", "Stop serial_bridge")}
                </button>
                <button className="connection-button btn-connect" onClick={refreshSerialBridgeLogs}>
                  {tr("ログ更新", "Refresh Logs")}
                </button>
                <button
                  className={`serial-periodic-button ${serialBridgeLogRealtimeEnabled ? "serial-periodic-on" : ""}`}
                  onClick={() => setSerialBridgeLogRealtimeEnabled((prev) => !prev)}
                >
                  {serialBridgeLogRealtimeEnabled ? tr("ログ自動更新: ON", "Auto Log: ON") : tr("ログ自動更新: OFF", "Auto Log: OFF")}
                </button>
                <span className="connection-hint">{serialBridgeLoading ? translateRuntimeText("処理中...") : translateRuntimeText(serialBridgeInfo)}</span>
              </div>

              <div className="serial-bridge-grid">
                <section className="serial-bridge-card">
                  <h3 className="serial-bridge-title">{tr("実行状態", "Runtime")}</h3>
                  <p className="connection-hint">running: {serialBridgeRunning ? "ON" : "OFF"}</p>
                  <p className="connection-hint">pid: {serialBridgePid || "-"}</p>
                </section>

                <section className="serial-bridge-card">
                  <h3 className="serial-bridge-title">{tr("検出ポート", "Detected Ports")}</h3>
                  <div className="serial-bridge-list">
                    {serialBridgePorts.length === 0 && <p className="connection-hint">{translateRuntimeText("ポート未検出")}</p>}
                    {serialBridgePorts.map((port) => (
                      <div className="serial-bridge-list-row" key={port}>{port}</div>
                    ))}
                  </div>
                </section>

                <section className="serial-bridge-card">
                  <h3 className="serial-bridge-title">{tr("検出ID (serial_rx/tx)", "Detected IDs (serial_rx/tx)")}</h3>
                  <div className="serial-bridge-list">
                    {serialBridgeIds.length === 0 && <p className="connection-hint">{translateRuntimeText("ID未検出")}</p>}
                    {serialBridgeIds.map((id) => (
                      <div className="serial-bridge-list-row" key={`id-${id}`}>{tr("ID", "ID")}: {id}</div>
                    ))}
                  </div>
                </section>

                <section className="serial-bridge-card serial-bridge-card-log">
                  <h3 className="serial-bridge-title">{tr("serial_bridge ログ", "serial_bridge Logs")}</h3>
                  <div className="serial-bridge-log-box" ref={serialBridgeLogBoxRef}>
                    {serialBridgeLogLoading && <p className="connection-hint">{translateRuntimeText("ログ取得中...")}</p>}
                    {!serialBridgeLogLoading && serialBridgeLogs.length === 0 && (
                      <p className="connection-hint">{translateRuntimeText("ログはまだありません")}</p>
                    )}
                    {!serialBridgeLogLoading && serialBridgeLogs.length > 0 && (
                      <pre className="serial-bridge-log-pre">{serialBridgeLogs.join("\n")}</pre>
                    )}
                  </div>
                </section>
              </div>
            </section>
          )}

          {isPageActive("shutdown") && (
            <section className="serial-bridge-panel">
              <h2 className="serial-packet-title">{tr("強制停止", "Shutdown")}</h2>
              <p className="serial-packet-hint">
                {tr("フロントエンドとバックエンドの強制停止操作を行います。実行すると復帰に再起動/再読み込みが必要です。強制停止後の機体の動作は保証できません。", "Force-stop frontend/backend processes. Recovery requires reload/restart. Robot behavior is not guaranteed after force shutdown.")}
              </p>

              <div className="serial-bridge-toolbar">
                <button className="connection-button btn-neutral" onClick={forceShutdownFrontendFromConsole}>
                  {tr("frontend 強制停止", "Force Stop Frontend")}
                </button>
                <button className="connection-button btn-neutral" onClick={forceShutdownBackendFromConsole}>
                  {tr("backend 強制停止", "Force Stop Backend")}
                </button>
                <span className="connection-hint">{serialBridgeLoading ? translateRuntimeText("処理中...") : translateRuntimeText(serialBridgeInfo)}</span>
              </div>
            </section>
          )}

          {isPageActive("settings") && (
            <section className="serial-bridge-panel">
              <h2 className="serial-packet-title">{tr("設定", "Settings")}</h2>
              <p className="serial-packet-hint">
                {tr("接続先、操作ロック、送信設定、ログ設定を一括管理します。", "Manage connection, safety lock, transmission, and log settings.")}
              </p>

              <div className="serial-bridge-grid">
                <section className="serial-bridge-card">
                  <h3 className="serial-bridge-title">{tr("接続設定", "Connection")}</h3>
                  <div className="serial-bridge-list">
                    <label className="serial-packet-label">
                      ROS Host
                      <input
                        className="connection-input"
                        value={rosHostInput}
                        onChange={(e) => setRosHostInput(e.target.value)}
                      />
                    </label>
                    <label className="serial-packet-label">
                      ROS Port
                      <input
                        className="connection-input"
                        value={rosPortInput}
                        onChange={(e) => setRosPortInput(e.target.value)}
                      />
                    </label>
                    <label className="serial-packet-label">
                      Joy Topic
                      <input
                        className="connection-input"
                        value={joyTopicInput}
                        onChange={(e) => setJoyTopicInput(e.target.value)}
                      />
                    </label>
                  </div>
                </section>

                <section className="serial-bridge-card">
                  <h3 className="serial-bridge-title">{tr("操作設定", "Operation")}</h3>
                  <div className="serial-bridge-list">
                    <div className="lang-switch-row">
                      {renderLanguageSelect("lang-select lang-select-wide", "lang-settings")}
                    </div>
                    <button
                      className={`toggle-button ${operationArmed ? "toggle-on" : "toggle-off"}`}
                      onClick={() => {
                        const next = !operationArmed;
                        setOperationArmed(next);
                        if (!next) {
                          resetAllControls();
                        }
                      }}
                    >
                      {operationArmed ? tr("操作ロック: OFF", "Safety Lock: OFF") : tr("操作ロック: ON", "Safety Lock: ON")}
                    </button>
                    <button
                      className={`toggle-button ${controllerEnabled ? "toggle-on" : "toggle-off"}`}
                      onClick={() => setControllerEnabled((prev) => !prev)}
                    >
                      {controllerEnabled ? tr("コントローラー: ON", "Controller: ON") : tr("コントローラー: OFF", "Controller: OFF")}
                    </button>
                    <button
                      className={`serial-periodic-button ${serialBridgeLogRealtimeEnabled ? "serial-periodic-on" : ""}`}
                      onClick={() => setSerialBridgeLogRealtimeEnabled((prev) => !prev)}
                    >
                      {serialBridgeLogRealtimeEnabled ? tr("ログ自動更新: ON", "Auto Log: ON") : tr("ログ自動更新: OFF", "Auto Log: OFF")}
                    </button>
                  </div>
                </section>

                <section className="serial-bridge-card">
                  <h3 className="serial-bridge-title">{tr("グラフ設定", "Graph Settings")}</h3>
                  <div className="serial-bridge-list">
                    <label className="serial-packet-label">
                      {tr("Yaw オフセット (°)", "Yaw Offset (°)")}
                      <input
                        className="connection-input"
                        type="number"
                        step="1"
                        value={yawOffsetDegInput}
                        onChange={(e) => setYawOffsetDegInput(e.target.value)}
                      />
                    </label>
                    <p className="connection-hint">{tr("すべてのグラフにおける姿勢表示を調整します (0〜360°)", "Adjusts yaw orientation in all graphs (0-360°)")}</p>
                  </div>
                </section>

                <section className="serial-bridge-card">
                  <h3 className="serial-bridge-title">{tr("送信設定", "Transmission")}</h3>
                  <div className="serial-bridge-list">
                    <label className="serial-packet-label">
                      Serial Target ID
                      <input
                        className="connection-input"
                        type="number"
                        min="0"
                        max="255"
                        value={serialTargetIdInput}
                        onChange={(e) => setSerialTargetIdInput(e.target.value)}
                      />
                    </label>
                    <label className="serial-packet-label">
                      {tr("配列要素数", "Array Length")}
                      <input
                        className="connection-input"
                        type="number"
                        min="1"
                        max="64"
                        value={serialElementCount}
                        onChange={(e) => updateSerialElementCount(e.target.value)}
                      />
                    </label>
                    <label className="serial-packet-label">
                      {tr("定期送信Hz", "Periodic Hz")}
                      <select
                        className="connection-input"
                        value={serialPeriodicHz}
                        onChange={(e) => setSerialPeriodicHz(e.target.value)}
                      >
                        <option value="1">1</option>
                        <option value="2">2</option>
                        <option value="5">5</option>
                        <option value="10">10</option>
                        <option value="20">20</option>
                        <option value="30">30</option>
                        <option value="50">50</option>
                      </select>
                    </label>
                  </div>
                </section>

                <section className="serial-bridge-card">
                  <h3 className="serial-bridge-title">{tr("ログ設定", "Logs")}</h3>
                  <div className="serial-bridge-list">
                    <label className="serial-packet-label">
                      {tr("取得行数 (10-1000)", "Lines (10-1000)")}
                      <input
                        className="connection-input"
                        type="number"
                        min="10"
                        max="1000"
                        value={serialBridgeLogLinesInput}
                        onChange={(e) => setSerialBridgeLogLinesInput(e.target.value)}
                      />
                    </label>
                    <p className="connection-hint">{tr("現在の取得行数:", "Current lines:")} {serialBridgeLogLineLimit}</p>
                  </div>
                </section>
              </div>

              <div className="serial-bridge-toolbar">
                <button className="connection-button btn-connect" onClick={applySettingsValues}>
                  {tr("設定を適用", "Apply Settings")}
                </button>
                <button className="connection-button btn-neutral" onClick={refreshSerialBridgeStatus}>
                  {tr("状態を再取得", "Refresh Status")}
                </button>
                <span className="connection-hint">{serialBridgeLoading ? translateRuntimeText("処理中...") : translateRuntimeText(serialBridgeInfo)}</span>
              </div>
            </section>
          )}
        </section>
      </main>
    </div>
  );
}

export default App;