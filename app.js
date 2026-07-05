// ==========================================
// 1. 全局变量与 3D 场景定义
// ==========================================
let scene, camera, renderer, controls;
let chestGroup, shoulderGroup, elbowGroup, handGroup;
let upperArmMesh, forearmMesh;
let upperSensorMesh, foreSensorMesh; // 3D 传感器模型
let debugMarkerMesh; // 用于验证肘部坐标 (X_E, Y_E, Z_E) 的半透明橙色标记球
let debugWristMarkerMesh; // 用于验证手腕坐标 (X_W, Y_W, Z_W) 的半透明粉色标记球

// 肢体与安装参数配置全局变量 (单位: cm)
let armConfig = {
    upperLength: 30,
    forearmLength: 26,
    upperSensorPos: 5,
    forearmSensorPos: 3
};

// 实时原始数据缓冲 (包含四元数分量，Module 1)
let rawData = {
    upper: { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0, r: 0, p: 0, y: 0, q0: 1, q1: 0, q2: 0, q3: 0, connected: false },
    fore: { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0, r: 0, p: 0, y: 0, q0: 1, q1: 0, q2: 0, q3: 0, connected: false }
};

// ==========================================
// 2. 双节点标定状态机常量与变量定义 (Module 2)
// ==========================================
const WINDOW_SIZE = 25;
const LOCK_FRAMES = 20;

const STATE_IDLE = 'STATE_IDLE';
const STATE_CHECKING = 'STATE_CHECKING';
const STATE_LOCKING = 'STATE_LOCKING';
const STATE_CALIBRATED = 'STATE_CALIBRATED';

let calibState = STATE_IDLE;

// 独立的角速度滑动窗口
let gyroWindowUpper = [];
let gyroWindowFore = [];

// 双节点锁定变量
let lockCounter = 0;
let lockSumUpper = { q0: 0, q1: 0, q2: 0, q3: 0 };
let lockSumFore = { q0: 0, q1: 0, q2: 0, q3: 0 };
let lockFirstUpper = null;
let lockFirstFore = null;

// 双标定偏差偏移量 (Q_ref_conj，Module 2 & 3)
let offsets = {
    upper: { q0: 1, q1: 0, q2: 0, q3: 0, active: false },
    fore: { q0: 1, q1: 0, q2: 0, q3: 0, active: false }
};

// 蓝牙服务 UUID 定义
const WIT_SERVICE_UUIDS = [
    '0000ffe5-0000-1000-8000-00805f9b34fb',
    '0000ffe5-0000-1000-8000-00805f9a34fb',
    '0000ffe0-0000-1000-8000-00805f9b34fb',
    '0000ffe9-0000-1000-8000-00805f9b34fb',
    '0000ffe7-0000-1000-8000-00805f9b34fb',
    '49535343-fe7d-4ae5-8fa9-9fafd205e455'
];

// ==========================================
// 3. 初始化 3D 渲染场景
// ==========================================
function init3D() {
    const holder = document.getElementById('canvas-holder');
    const width = holder.clientWidth;
    const height = holder.clientHeight;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0c0d12);

    camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 10);
    camera.position.set(0.6, 1.3, 1.6);
    camera.lookAt(0, 1.2, 0);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    holder.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.maxPolarAngle = Math.PI / 2 + 0.15;
    controls.target.set(0, 1.2, 0);
    controls.update();

    // 光照系统
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
    scene.add(ambientLight);

    const hemiLight = new THREE.HemisphereLight(0xffffff, 0x444444, 0.6);
    hemiLight.position.set(0, 5, 0);
    scene.add(hemiLight);

    const dirLight1 = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight1.position.set(3, 5, 4);
    scene.add(dirLight1);

    const dirLight2 = new THREE.DirectionalLight(0xffffff, 0.4);
    dirLight2.position.set(-3, 3, -3);
    scene.add(dirLight2);

    const pointLight = new THREE.PointLight(0x00f2fe, 1.2, 8);
    pointLight.position.set(0, 1.5, 0.8);
    scene.add(pointLight);

    const gridHelper = new THREE.GridHelper(2, 20, 0x334155, 0x1e293b);
    gridHelper.position.y = 0.5;
    scene.add(gridHelper);

    // ==========================================
    // 搭建骨骼模型 (大臂小臂绑定姿态链)
    // ==========================================
    // 1. 身体躯干 (Chest)
    chestGroup = new THREE.Group();
    chestGroup.position.set(0, 1.2, 0);
    scene.add(chestGroup);

    const torsoGeo = new THREE.BoxGeometry(0.24, 0.45, 0.12);
    const torsoMat = new THREE.MeshStandardMaterial({ color: 0x334155, roughness: 0.4, metalness: 0.2 });
    const torsoMesh = new THREE.Mesh(torsoGeo, torsoMat);
    chestGroup.add(torsoMesh);

    // 2. 右肩关节 (Shoulder)
    shoulderGroup = new THREE.Group();
    shoulderGroup.position.set(-0.15, 0.2, 0); // 肩膀偏置
    chestGroup.add(shoulderGroup);

    const jointGeo = new THREE.SphereGeometry(0.04, 32, 32);
    const jointMatCyan = new THREE.MeshStandardMaterial({ color: 0x00f2fe, emissive: 0x00f2fe, emissiveIntensity: 0.8 });
    const shoulderJoint = new THREE.Mesh(jointGeo, jointMatCyan);
    shoulderGroup.add(shoulderJoint);

    // 大臂骨骼
    const upperArmGeo = new THREE.CylinderGeometry(0.02, 0.016, 0.3, 16);
    upperArmGeo.translate(0, -0.15, 0);
    const boneMat = new THREE.MeshStandardMaterial({ color: 0x00d2ff, roughness: 0.3, metalness: 0.3 });
    upperArmMesh = new THREE.Mesh(upperArmGeo, boneMat);
    shoulderGroup.add(upperArmMesh);

    // 传感器材质与尺寸
    const sensorMat = new THREE.MeshStandardMaterial({
        color: 0x39ff14,
        emissive: 0x39ff14,
        emissiveIntensity: 0.6,
        roughness: 0.3,
        metalness: 0.2
    });
    const sensorGeo = new THREE.BoxGeometry(0.03, 0.04, 0.02);

    // 大臂传感器 (绑定在大臂组)
    upperSensorMesh = new THREE.Mesh(sensorGeo, sensorMat);
    upperSensorMesh.position.set(-0.03, -0.25, 0);
    shoulderGroup.add(upperSensorMesh);

    // 3. 肘关节 (Elbow)
    elbowGroup = new THREE.Group();
    elbowGroup.position.set(0, -0.3, 0);
    shoulderGroup.add(elbowGroup);

    const elbowJoint = new THREE.Mesh(jointGeo, jointMatCyan);
    elbowGroup.add(elbowJoint);

    // 小臂骨骼
    const forearmGeo = new THREE.CylinderGeometry(0.016, 0.012, 0.26, 16);
    forearmGeo.translate(0, -0.13, 0);
    forearmMesh = new THREE.Mesh(forearmGeo, boneMat);
    elbowGroup.add(forearmMesh);

    // 小臂传感器 (绑定在小臂组)
    foreSensorMesh = new THREE.Mesh(sensorGeo, sensorMat);
    foreSensorMesh.position.set(-0.025, -0.23, 0);
    elbowGroup.add(foreSensorMesh);

    // 4. 手部 (Hand)
    handGroup = new THREE.Group();
    handGroup.position.set(0, -0.26, 0);
    elbowGroup.add(handGroup);

    const handGeo = new THREE.SphereGeometry(0.025, 16, 16);
    const jointMatPink = new THREE.MeshStandardMaterial({ color: 0xff007f, emissive: 0xff007f, emissiveIntensity: 0.8 });
    const handMesh = new THREE.Mesh(handGeo, jointMatPink);
    handGroup.add(handMesh);

    // ==========================================
    // 5. 挂载两个验证简化公式 (Module 4) 的半透明标记球
    // ==========================================
    // 肘部位置半透明橙色球
    const markerGeo1 = new THREE.SphereGeometry(0.042, 32, 32);
    const markerMat1 = new THREE.MeshStandardMaterial({
        color: 0xffa500, // 橙色
        transparent: true,
        opacity: 0.6,
        roughness: 0.3,
        metalness: 0.1
    });
    debugMarkerMesh = new THREE.Mesh(markerGeo1, markerMat1);
    debugMarkerMesh.position.set(-0.15, 0.2 - 0.3, 0);
    chestGroup.add(debugMarkerMesh);

    // 手腕位置半透明粉色球
    const markerGeo2 = new THREE.SphereGeometry(0.038, 32, 32);
    const markerMat2 = new THREE.MeshStandardMaterial({
        color: 0xff00ff, // 粉色
        transparent: true,
        opacity: 0.6,
        roughness: 0.3,
        metalness: 0.1
    });
    debugWristMarkerMesh = new THREE.Mesh(markerGeo2, markerMat2);
    debugWristMarkerMesh.position.set(-0.15, 0.2 - 0.56, 0);
    chestGroup.add(debugWristMarkerMesh);

    window.addEventListener('resize', onWindowResize);
    animate();
}

function onWindowResize() {
    const holder = document.getElementById('canvas-holder');
    const width = holder.clientWidth;
    const height = holder.clientHeight;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
}

function animate() {
    requestAnimationFrame(animate);
    if (controls) controls.update();
    renderer.render(scene, camera);
}

// ==========================================
// 4. Web Bluetooth 数据帧解析 (Module 1)
// ==========================================
async function connectSensor(type) {
    const statusEl = document.getElementById(`status-${type}`);
    const btnEl = document.getElementById(`btn-connect-${type}`);

    try {
        statusEl.innerText = "正在搜索...";
        statusEl.className = "status-badge status-disconnected";
        statusEl.style.color = "#eab308";

        const device = await navigator.bluetooth.requestDevice({
            filters: [
                { namePrefix: 'WT901BLE' },
                { namePrefix: 'BWT901BLE' }
            ],
            optionalServices: WIT_SERVICE_UUIDS
        });

        statusEl.innerText = "正在连接 GATT...";
        const server = await device.gatt.connect();

        statusEl.innerText = "正在搜寻特征通道...";
        
        let service = null;
        let characteristic = null;

        for (const serviceUuid of WIT_SERVICE_UUIDS) {
            try {
                service = await server.getPrimaryService(serviceUuid);
                const characteristics = await service.getCharacteristics();
                for (const char of characteristics) {
                    if (char.properties.notify) {
                        characteristic = char;
                        break;
                    }
                }
                if (characteristic) break;
            } catch (e) {
                // 继续
            }
        }

        if (!service || !characteristic) {
            throw new Error("找不到匹配的维特数据通知通道");
        }

        await characteristic.startNotifications();
        
        characteristic.addEventListener('characteristicvaluechanged', (event) => {
            const dataView = event.target.value;
            const len = dataView.byteLength;
            
            // 校验头部 (0x55 0x61 角度/混合数据包，Module 1)
            if (len >= 20 && dataView.getUint8(0) === 0x55 && dataView.getUint8(1) === 0x61) {
                let axRaw = dataView.getInt16(2, true);
                let ayRaw = dataView.getInt16(4, true);
                let azRaw = dataView.getInt16(6, true);
                let gxRaw = dataView.getInt16(8, true);
                let gyRaw = dataView.getInt16(10, true);
                let gzRaw = dataView.getInt16(12, true);
                let rollRaw = dataView.getInt16(14, true);
                let pitchRaw = dataView.getInt16(16, true);
                let yawRaw = dataView.getInt16(18, true);

                const ax = axRaw / 32768.0 * 16.0;   // g
                const ay = ayRaw / 32768.0 * 16.0;   // g
                const az = azRaw / 32768.0 * 16.0;   // g
                const gx = gxRaw / 32768.0 * 2000.0; // deg/s
                const gy = gyRaw / 32768.0 * 2000.0; // deg/s
                const gz = gzRaw / 32768.0 * 2000.0; // deg/s
                const rDeg = rollRaw / 32768.0 * 180.0;
                const pDeg = pitchRaw / 32768.0 * 180.0;
                const yDeg = yawRaw / 32768.0 * 180.0;

                // 构建归一化当前四元数 Q_curr
                const q_curr_three = new THREE.Quaternion().setFromEuler(new THREE.Euler(
                    THREE.MathUtils.degToRad(rDeg), // Roll (X)
                    THREE.MathUtils.degToRad(pDeg), // Pitch (Y)
                    THREE.MathUtils.degToRad(yDeg), // Yaw (Z)
                    'ZYX'
                ));

                // 写入数据结构
                rawData[type] = {
                    ax, ay, az,
                    gx, gy, gz,
                    r: rDeg, p: pDeg, y: yDeg,
                    q0: q_curr_three.w,
                    q1: q_curr_three.x,
                    q2: q_curr_three.y,
                    q3: q_curr_three.z,
                    connected: true
                };

                // 5. 驱动同步标定状态机运行 (Module 2)
                runCalibrationStateMachine();

                // 6. 数据显示逻辑 (标定后显示消差后的 Euler 姿态)
                let rDisplay = rDeg;
                let pDisplay = pDeg;
                let yDisplay = yDeg;

                if (offsets[type].active) {
                    const q_w = getWorkingQuaternion(type);
                    const q_w_three = new THREE.Quaternion(q_w.q1, q_w.q2, q_w.q3, q_w.q0);
                    const euler_w = new THREE.Euler().setFromQuaternion(q_w_three, 'ZYX');
                    rDisplay = THREE.MathUtils.radToDeg(euler_w.x);
                    pDisplay = THREE.MathUtils.radToDeg(euler_w.y);
                    yDisplay = THREE.MathUtils.radToDeg(euler_w.z);
                }

                document.getElementById(`val-${type}-r`).innerText = rDisplay.toFixed(1);
                document.getElementById(`val-${type}-p`).innerText = pDisplay.toFixed(1);
                document.getElementById(`val-${type}-y`).innerText = yDisplay.toFixed(1);

                // 实时渲染姿态链与正解链
                update3DModel();
            }
        });

        statusEl.innerText = "已连接";
        statusEl.className = "status-badge status-connected";
        statusEl.style.color = "";
        btnEl.innerText = "断开 (请刷新)";
        btnEl.className = "btn-connect connected";
        btnEl.disabled = true;

    } catch (err) {
        console.error("连接失败: ", err);
        statusEl.innerText = "失败: " + err.message;
        statusEl.className = "status-badge status-disconnected";
        statusEl.style.color = "";
    }
}

// ==========================================
// 5. 严格的同步双节点标定状态机 (Module 2)
// ==========================================
function calculateGyroVariance(windowArr) {
    let sumX = 0, sumY = 0, sumZ = 0;
    for (let i = 0; i < WINDOW_SIZE; i++) {
        sumX += windowArr[i].gx;
        sumY += windowArr[i].gy;
        sumZ += windowArr[i].gz;
    }
    const meanX = sumX / WINDOW_SIZE;
    const meanY = sumY / WINDOW_SIZE;
    const meanZ = sumZ / WINDOW_SIZE;

    let varX = 0, varY = 0, varZ = 0;
    for (let i = 0; i < WINDOW_SIZE; i++) {
        varX += Math.pow(windowArr[i].gx - meanX, 2);
        varY += Math.pow(windowArr[i].gy - meanY, 2);
        varZ += Math.pow(windowArr[i].gz - meanZ, 2);
    }
    return (varX + varY + varZ) / WINDOW_SIZE;
}

function runCalibrationStateMachine() {
    // 两个节点未连齐时，直接返回
    if (!rawData.upper.connected || !rawData.fore.connected) {
        return;
    }

    // 压入大臂/小臂角速度滑动窗口
    gyroWindowUpper.push({ gx: rawData.upper.gx, gy: rawData.upper.gy, gz: rawData.upper.gz });
    if (gyroWindowUpper.length > WINDOW_SIZE) gyroWindowUpper.shift();

    gyroWindowFore.push({ gx: rawData.fore.gx, gy: rawData.fore.gy, gz: rawData.fore.gz });
    if (gyroWindowFore.length > WINDOW_SIZE) gyroWindowFore.shift();

    if (calibState === STATE_IDLE || calibState === STATE_CALIBRATED) {
        return;
    }

    if (gyroWindowUpper.length < WINDOW_SIZE || gyroWindowFore.length < WINDOW_SIZE) {
        return;
    }

    // 1. 条件一：计算双节点陀螺仪方差
    const g_var_upper = calculateGyroVariance(gyroWindowUpper);
    const g_var_fore = calculateGyroVariance(gyroWindowFore);
    
    // 静止门限 < 3.0 (根据实际传感器抖动调整)
    const isStillUpper = (g_var_upper < 3.0);
    const isStillFore = (g_var_fore < 3.0);

    // 2. 条件二：垂直检测 (重力 ay 门限 1.0g +/- 0.5g)
    const isVerticalUpper = (Math.abs(rawData.upper.ax) < 0.5) && 
                            (Math.abs(rawData.upper.az) < 0.5) && 
                            (rawData.upper.ay > 0.5 && rawData.upper.ay < 1.5);
                            
    const isVerticalFore = (Math.abs(rawData.fore.ax) < 0.5) && 
                           (Math.abs(rawData.fore.az) < 0.5) && 
                           (rawData.fore.ay > 0.5 && rawData.fore.ay < 1.5);

    // 实时更新前端仪表数据
    document.getElementById('val-gvar-upper').innerText = g_var_upper.toFixed(3);
    document.getElementById('val-ay-upper').innerText = rawData.upper.ay.toFixed(3);
    document.getElementById('val-gvar-fore').innerText = g_var_fore.toFixed(3);
    document.getElementById('val-ay-fore').innerText = rawData.fore.ay.toFixed(3);

    const statusText = document.getElementById('calibration-status');

    // 3. 状态跃迁
    if (calibState === STATE_CHECKING) {
        statusText.style.color = "#f59e0b";
        statusText.innerText = "状态: 检测静止与垂直中 (STATE_CHECKING)";

        // 必须同时满足大臂、小臂的静止+垂直
        if (isStillUpper && isVerticalUpper && isStillFore && isVerticalFore) {
            calibState = STATE_LOCKING;
            lockCounter = 0;
            lockSumUpper = { q0: 0, q1: 0, q2: 0, q3: 0 };
            lockSumFore = { q0: 0, q1: 0, q2: 0, q3: 0 };
            lockFirstUpper = null;
            lockFirstFore = null;
        }
    } 
    else if (calibState === STATE_LOCKING) {
        statusText.style.color = "#3b82f6";
        statusText.innerText = `状态: 锁定零点中 (STATE_LOCKING)... 进度: ${lockCounter}/20`;

        // 任何一环打破了垂直/静止，立刻退回 CHECKING 重新检测
        if (!isStillUpper || !isVerticalUpper || !isStillFore || !isVerticalFore) {
            calibState = STATE_CHECKING;
            lockCounter = 0;
            return;
        }

        // 第一帧数据锁存
        if (lockCounter === 0) {
            lockFirstUpper = { q0: rawData.upper.q0, q1: rawData.upper.q1, q2: rawData.upper.q2, q3: rawData.upper.q3 };
            lockFirstFore = { q0: rawData.fore.q0, q1: rawData.fore.q1, q2: rawData.fore.q2, q3: rawData.fore.q3 };
        }

        // 大臂点积符号校正
        const dotUpper = rawData.upper.q0 * lockFirstUpper.q0 +
                         rawData.upper.q1 * lockFirstUpper.q1 +
                         rawData.upper.q2 * lockFirstUpper.q2 +
                         rawData.upper.q3 * lockFirstUpper.q3;
        const signUpper = (dotUpper < 0.0) ? -1.0 : 1.0;

        lockSumUpper.q0 += rawData.upper.q0 * signUpper;
        lockSumUpper.q1 += rawData.upper.q1 * signUpper;
        lockSumUpper.q2 += rawData.upper.q2 * signUpper;
        lockSumUpper.q3 += rawData.upper.q3 * signUpper;

        // 小臂点积符号校正
        const dotFore = rawData.fore.q0 * lockFirstFore.q0 +
                        rawData.fore.q1 * lockFirstFore.q1 +
                        rawData.fore.q2 * lockFirstFore.q2 +
                        rawData.fore.q3 * lockFirstFore.q3;
        const signFore = (dotFore < 0.0) ? -1.0 : 1.0;

        lockSumFore.q0 += rawData.fore.q0 * signFore;
        lockSumFore.q1 += rawData.fore.q1 * signFore;
        lockSumFore.q2 += rawData.fore.q2 * signFore;
        lockSumFore.q3 += rawData.fore.q3 * signFore;

        lockCounter++;

        // 锁定结束，双节点归一化共轭锁定 (Module 2)
        if (lockCounter >= LOCK_FRAMES) {
            const normUpper = Math.sqrt(
                lockSumUpper.q0 * lockSumUpper.q0 +
                lockSumUpper.q1 * lockSumUpper.q1 +
                lockSumUpper.q2 * lockSumUpper.q2 +
                lockSumUpper.q3 * lockSumUpper.q3
            );
            const normFore = Math.sqrt(
                lockSumFore.q0 * lockSumFore.q0 +
                lockSumFore.q1 * lockSumFore.q1 +
                lockSumFore.q2 * lockSumFore.q2 +
                lockSumFore.q3 * lockSumFore.q3
            );

            if (normUpper > 0.0 && normFore > 0.0) {
                // 锁定大臂 Q_ref_conj
                offsets.upper.q0 = lockSumUpper.q0 / normUpper;
                offsets.upper.q1 = -(lockSumUpper.q1 / normUpper);
                offsets.upper.q2 = -(lockSumUpper.q2 / normUpper);
                offsets.upper.q3 = -(lockSumUpper.q3 / normUpper);
                offsets.upper.active = true;

                // 锁定小臂 Q_ref_conj
                offsets.fore.q0 = lockSumFore.q0 / normFore;
                offsets.fore.q1 = -(lockSumFore.q1 / normFore);
                offsets.fore.q2 = -(lockSumFore.q2 / normFore);
                offsets.fore.q3 = -(lockSumFore.q3 / normFore);
                offsets.fore.active = true;

                calibState = STATE_CALIBRATED;
                statusText.style.color = "#10b981";
                statusText.innerText = "状态: 双节点标定完成 (STATE_CALIBRATED)";
            } else {
                calibState = STATE_CHECKING;
            }
        }
    }
}

function triggerAutoCalibration() {
    calibState = STATE_CHECKING;
    gyroWindowUpper = [];
    gyroWindowFore = [];
    lockCounter = 0;
    lockSumUpper = { q0: 0, q1: 0, q2: 0, q3: 0 };
    lockSumFore = { q0: 0, q1: 0, q2: 0, q3: 0 };
    lockFirstUpper = null;
    lockFirstFore = null;
    offsets.upper.active = false;
    offsets.fore.active = false;

    const statusText = document.getElementById('calibration-status');
    statusText.style.color = "#f59e0b";
    statusText.innerText = "状态: 检测静止与垂直中 (STATE_CHECKING)";
}

// ==========================================
// 6. 工作四元数解算 (Module 3)
// ==========================================
function getWorkingQuaternion(type) {
    const c = rawData[type];
    if (!offsets[type].active) {
        return { q0: c.q0, q1: c.q1, q2: c.q2, q3: c.q3 };
    }
    const r = offsets[type]; // Q_ref_conj

    let q_work = {};
    // q_work = q_ref_conj * q_raw
    q_work.q0 = r.q0 * c.q0 - r.q1 * c.q1 - r.q2 * c.q2 - r.q3 * c.q3;
    q_work.q1 = r.q0 * c.q1 + r.q1 * c.q0 + r.q2 * c.q3 - r.q3 * c.q2;
    q_work.q2 = r.q0 * c.q2 - r.q1 * c.q3 + r.q2 * c.q0 + r.q3 * c.q1;
    q_work.q3 = r.q0 * c.q3 + r.q1 * c.q2 - r.q2 * c.q1 + r.q3 * c.q0;

    return q_work;
}

// ==========================================
// 7. 双节正向运动链与 3D 映射 (Module 4)
// ==========================================
function update3DModel() {
    if (!shoulderGroup || !elbowGroup || !debugMarkerMesh || !debugWristMarkerMesh) return;

    // 1. 分别解算大臂与小臂工作四元数 (Module 3)
    const upper = getWorkingQuaternion('upper');
    const fore = getWorkingQuaternion('fore');

    // 2. 将双工作四元数重建为 Three.js 四元数格式
    const q_upper_work_three = new THREE.Quaternion(upper.q1, upper.q2, upper.q3, upper.q0);
    const q_fore_work_three = new THREE.Quaternion(fore.q1, fore.q2, fore.q3, fore.q0);

    // 3. 构建大臂与小臂的绝对世界朝向 (叠加 -90度 绕垂直 Y 轴修正量)
    const q_yaw_offset = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -Math.PI / 2);
    const q_upper_abs = q_yaw_offset.clone().multiply(q_upper_work_three);
    const q_fore_abs = q_yaw_offset.clone().multiply(q_fore_work_three);

    // 4. 将姿态解算链应用在 3D 模型骨架中 (父子级层级旋转链)
    // 大臂直接旋转其绝对姿态
    shoulderGroup.quaternion.copy(q_upper_abs);
    // 小臂的本地相对旋转 = 大臂绝对旋转的逆 * 小臂绝对旋转 (Elbow_Local_Q = Shoulder_Abs_Q_inv * Forearm_Abs_Q)
    elbowGroup.quaternion.copy(q_upper_abs.clone().invert().multiply(q_fore_abs));

    // 5. 运行双节运动学正解链 (基于 Y 轴模型 V_b = [0, -Length, 0] 的公式解算，Module 4)
    const L1 = armConfig.upperLength / 100;    // 大臂长 (米)
    const L2 = armConfig.forearmLength / 100;  // 小臂长 (米)

    // 大臂空间向量 (肘关节坐标 P_E)
    const X_E = 2.0 * L1 * (upper.q0 * upper.q3 - upper.q1 * upper.q2);
    const Y_E = L1 * (2.0 * (upper.q1 * upper.q1 + upper.q3 * upper.q3) - 1.0);
    const Z_E = -2.0 * L1 * (upper.q0 * upper.q1 + upper.q2 * upper.q3);

    // 小臂空间向量 (Vector_Forearm dX, dY, dZ)
    const dX = 2.0 * L2 * (fore.q0 * fore.q3 - fore.q1 * fore.q2);
    const dY = L2 * (2.0 * (fore.q1 * fore.q1 + fore.q3 * fore.q3) - 1.0);
    const dZ = -2.0 * L2 * (fore.q0 * fore.q1 + fore.q2 * fore.q3);

    // 6. 将正解坐标同样进行 -90 度绕世界 Y 轴旋转（顺时针旋转90度）以维持和 3D 朝向一致
    // X_rot = Z_old, Z_rot = -X_old
    const X_E_rot = Z_E;
    const Z_E_rot = -X_E;

    const dX_rot = dZ;
    const dZ_rot = -dX;

    // 矢量加法叠加求得手腕坐标 P_W (Module 4)
    const X_W_rot = X_E_rot + dX_rot;
    const Y_W = Y_E + dY;
    const Z_W_rot = Z_E_rot + dZ_rot;

    // 7. 将解算出的肘部与手腕坐标显示在左侧面板中
    document.getElementById('val-coord-x').innerText = X_E_rot.toFixed(3);
    document.getElementById('val-coord-y').innerText = Y_E.toFixed(3);
    document.getElementById('val-coord-z').innerText = Z_E_rot.toFixed(3);

    document.getElementById('val-wrist-x').innerText = X_W_rot.toFixed(3);
    document.getElementById('val-wrist-y').innerText = Y_W.toFixed(3);
    document.getElementById('val-wrist-z').innerText = Z_W_rot.toFixed(3);

    // 8. 实时更新两个半透明辅助标记球的位置
    // 肩关节在躯干上的相对原点是 (-0.15, 0.2, 0)
    debugMarkerMesh.position.set(-0.15 + X_E_rot, 0.2 + Y_E, Z_E_rot);
    debugWristMarkerMesh.position.set(-0.15 + X_W_rot, 0.2 + Y_W, Z_W_rot);
}

// ==========================================
// 8. 肢体长度动态调整函数
// ==========================================
function update3DBoneLengths() {
    if (!upperArmMesh || !forearmMesh || !elbowGroup || !handGroup || !debugMarkerMesh || !debugWristMarkerMesh) return;
    
    const uLenMeters = armConfig.upperLength / 100;
    upperArmMesh.scale.set(1, uLenMeters / 0.3, 1);
    elbowGroup.position.y = -uLenMeters;
    
    const fLenMeters = armConfig.forearmLength / 100;
    forearmMesh.scale.set(1, fLenMeters / 0.26, 1);
    handGroup.position.y = -fLenMeters;

    if (upperSensorMesh) {
        const dUpperMeters = armConfig.upperSensorPos / 100;
        upperSensorMesh.position.set(-0.03, -uLenMeters + dUpperMeters, 0);
    }

    if (foreSensorMesh) {
        const dForeMeters = armConfig.forearmSensorPos / 100;
        foreSensorMesh.position.set(-0.025, -fLenMeters + dForeMeters, 0);
    }

    // 更新 3D 正解
    update3DModel();
}

function initConfigListeners() {
    document.getElementById('input-upper-len').addEventListener('input', (e) => {
        armConfig.upperLength = parseFloat(e.target.value) || 30;
        update3DBoneLengths();
    });
    document.getElementById('input-fore-len').addEventListener('input', (e) => {
        armConfig.forearmLength = parseFloat(e.target.value) || 26;
        update3DBoneLengths();
    });
    document.getElementById('input-upper-pos').addEventListener('input', (e) => {
        armConfig.upperSensorPos = parseFloat(e.target.value) || 5;
        update3DBoneLengths();
    });
    document.getElementById('input-fore-pos').addEventListener('input', (e) => {
        armConfig.forearmSensorPos = parseFloat(e.target.value) || 3;
        update3DBoneLengths();
    });
    
    update3DBoneLengths();
}

// ==========================================
// 9. 事件绑定与加载初始化
// ==========================================
window.onload = () => {
    init3D();
    initConfigListeners();

    document.getElementById('btn-connect-upper').addEventListener('click', () => {
        connectSensor('upper');
    });

    document.getElementById('btn-connect-fore').addEventListener('click', () => {
        connectSensor('fore');
    });

    document.getElementById('btn-calibrate-auto').addEventListener('click', () => {
        triggerAutoCalibration();
    });
};

// 辅助角校验函数
function getCalibratedAngle(raw, offset) {
    let diff = raw - offset;
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;
    return diff;
}
