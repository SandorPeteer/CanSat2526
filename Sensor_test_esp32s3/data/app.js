// AstroLink Air Quality Monitor
let ws, reconnectTimer, activityTimer;
let reconnect = true;
let lastActivityTime = 0;
const CONNECTION_TIMEOUT_MS = 45000;  // Consider connection dead after 45s no data
let currentPage = 'dashboard';
let lastImuTs = 0;
let zeroQuat = null;
let lastQuatNorm = null;
const autoZeroOnFirstSample = true;

const imuMapDefaults = {
    // UI axes (x right, y down, z toward viewer) in terms of sensor axes.
    xAxis: 'x',
    xSign: 1,
    yAxis: 'z',
    ySign: 1,
    zAxis: 'y',
    zSign: 1
};
const imuAxisMap = { ...imuMapDefaults };

// Chart state
let chartRange = 300;
let chartData = [];
let chartType = 0;  // 0=PM, 1=CO2, 2=Pressure, 3=Altitude, 4=Temperature, 5=Humidity, 6=Velocity, 7=Heading, 8=IMU
let chartLabels = [];
let chartUnit = 'µg/m³';
let chartSeriesCount = 4;
let chartCanvasId = 'chart';
let flightViewerActive = false;
const flightViewerCharts = {};
const flightViewerCanvasByType = [
    'flightChartPm',
    'flightChartCo2',
    'flightChartPress',
    'flightChartAlt',
    'flightChartTemp',
    'flightChartHum',
    'flightChartVel',
    'flightChartHead',
    'flightChartImu'
];
const flightChartLabelsMap = [['PM1','PM2.5','PM4','PM10'],['CO2'],['Pressure'],['Baro','GPS'],['BMP','SCD'],['Humidity'],['Velocity'],['Heading'],['Roll','Pitch','Yaw']];
const flightChartUnitsMap = ['µg/m³','ppm','hPa','m','°C','%','m/s','°','°'];
let liveMode = true;
let lastHistoryLoad = 0;
let flightChartTimer = null;

const $ = id => document.getElementById(id);
const fmt = v => v != null ? v.toFixed(1) : '--';
const fmt3 = v => v != null ? v.toFixed(3) : '--';
const fmt4 = v => v != null ? v.toFixed(4) : '--';
const wifiQualityLabel = rssi => {
    if (rssi == null || Number.isNaN(rssi)) return '--';
    if (rssi >= -55) return 'Excellent';
    if (rssi >= -65) return 'Good';
    if (rssi >= -75) return 'Fair';
    if (rssi >= -85) return 'Weak';
    return 'Very weak';
};
const imuAccText = ['Unreliable', 'Low', 'Medium', 'High'];
const radToDeg = 180 / Math.PI;

let bmpAltZero = null;
let lastBmpAlt = null;
let latestBno = null;
let latestCompass = null;
let latestBmp = null;
let latestScd = null;
let latestSps = null;
let latestGps = null;
let latestSys = null;
let latestTime = null;
let lastRender = {
    bno: 0,
    compass: 0,
    bmp: 0,
    scd: 0,
    sps: 0,
    gps: 0,
    sys: 0,
    time: 0
};

function loadBmpAltZero() {
    try {
        const raw = localStorage.getItem('bmpAltZero');
        if (raw == null) return;
        const val = parseFloat(raw);
        if (!Number.isNaN(val)) bmpAltZero = val;
    } catch (e) {}
}

function setBmpAltZero(val) {
    if (val == null) {
        toast('No BMP altitude yet', 'error');
        return;
    }
    bmpAltZero = val;
    try {
        localStorage.setItem('bmpAltZero', String(val));
    } catch (e) {}
    updateBmpAltZeroUI();
    toast('BMP altitude zeroed', 'success');
}

function updateBmpAltZeroUI() {
    const zeroText = bmpAltZero != null ? fmt4(bmpAltZero) : '--';
    if ($('bmp-alt-zero')) $('bmp-alt-zero').textContent = zeroText;
    if ($('bmp-alt-zero-big')) $('bmp-alt-zero-big').textContent = zeroText;
}

function initBmpAltZeroControls() {
    loadBmpAltZero();
    updateBmpAltZeroUI();

    const btnA = $('bmp-alt-zero-btn');
    if (btnA) btnA.addEventListener('click', () => setBmpAltZero(lastBmpAlt));
    const btnB = $('bmp-alt-zero-btn-big');
    if (btnB) btnB.addEventListener('click', () => setBmpAltZero(lastBmpAlt));
}

function handleBinaryMessage(buf) {
    const v = new DataView(buf);
    const type = v.getUint8(0);
    let o = 1;
    const readU8 = () => v.getUint8(o++);
    const readU16 = () => { const val = v.getUint16(o, true); o += 2; return val; };
    const readU32 = () => { const val = v.getUint32(o, true); o += 4; return val; };
    const readI16 = () => { const val = v.getInt16(o, true); o += 2; return val; };
    const readI32 = () => { const val = v.getInt32(o, true); o += 4; return val; };
    const readF32 = () => { const val = v.getFloat32(o, true); o += 4; return val; };

    switch (type) {
    case 1: {
        const qi = readF32();
        const qj = readF32();
        const qk = readF32();
        const qw = readF32();
        const acc = readF32();
        const accStatus = readU8();
        const ts = readU32();
        latestBno = { qw, qi, qj, qk, acc, accStatus, ts };
        break;
    }
    case 2: {
        const x = readI16();
        const y = readI16();
        const z = readI16();
        const heading = readF32();
        const ts = readU32();
        latestCompass = { x, y, z, heading, ts };
        break;
    }
    case 3: {
        const temp = readF32();
        const press = readF32();
        const alt = readF32();
        const ts = readU32();
        latestBmp = { temp, press, press_hpa: press / 100.0, alt, ts };
        break;
    }
    case 4: {
        const co2 = readF32();
        const temp = readF32();
        const hum = readF32();
        const ts = readU32();
        latestScd = { co2, temp, hum, ts };
        break;
    }
    case 5: {
        const pm1 = readF32();
        const pm25 = readF32();
        const pm4 = readF32();
        const pm10 = readF32();
        const nc05 = readF32();
        const nc1 = readF32();
        const nc25 = readF32();
        const nc4 = readF32();
        const nc10 = readF32();
        const size = readF32();
        const ts = readU32();
        latestSps = { pm1, pm25, pm4, pm10, nc05, nc1, nc25, nc4, nc10, size, ts };
        break;
    }
    case 6: {
        const itow = readU32();
        const year = readU16();
        const month = readU8();
        const day = readU8();
        const hour = readU8();
        const min = readU8();
        const sec = readU8();
        const valid = readU8();
        const tacc = readU32();
        const fix = readU8();
        const flags = readU8();
        const flags2 = readU8();
        const sat = readU8();
        const lonRaw = readI32();
        const latRaw = readI32();
        const height = readI32();
        const hmsl = readI32();
        const hacc = readU32();
        const vacc = readU32();
        const veln = readI32();
        const vele = readI32();
        const veld = readI32();
        const gspeed = readI32();
        const headingRaw = readI32();
        const sacc = readU32();
        const headacc = readU32();
        const pdop = readU16();
        const flags3 = readU8();
        const headVeh = readI32();
        const magDec = readI16();
        const magAcc = readU16();
        const ts = readU32();

        const pad = n => String(n).padStart(2, '0');
        const time = year ? `${year}-${pad(month)}-${pad(day)}T${pad(hour)}:${pad(min)}:${pad(sec)}` : '--';
        latestGps = {
            fix,
            sat,
            lat: latRaw / 1e7,
            lon: lonRaw / 1e7,
            alt: hmsl / 1000.0,
            height: height / 1000.0,
            speed: gspeed * 0.0036,
            heading: headingRaw * 1e-5,
            hacc,
            vacc,
            sacc,
            pdop,
            veln,
            vele,
            veld,
            flags,
            flags2,
            flags3,
            valid,
            itow,
            tacc,
            headacc,
            headVeh,
            magDec,
            magAcc,
            ts,
            time
        };
        break;
    }
    case 7: {
        const flags = readU8();
        const rssi = readI16();
        const temp = readF32();
        const ts = readU32();
        latestSys = { flags, rssi, temp, ts };
        break;
    }
    case 8: {
        const flags = readU8();
        const source = readU8();
        const offsetMin = readI16();
        const unix = readU32();
        latestTime = { flags, source, offsetMin, unix };
        break;
    }
    default:
        // Flight chart: 0xF0-0xF8 (PM, CO2, Pressure, Altitude, Temp, Humidity, Velocity, Heading, IMU)
        if (type >= 0xF0 && type <= 0xF8) {
            o = 1;
            const series = readU8();
            const count = readU16();
            const chartIdx = type - 0xF0;
            const labels = flightChartLabelsMap[chartIdx] || [];
            const unit = flightChartUnitsMap[chartIdx] || '';
            const data = [];
            for (let i = 0; i < count; i++) {
                const ts = readU32();
                const vals = [ts];
                for (let s = 0; s < series; s++) vals.push(readF32());
                data.push(vals);
            }
            lastHistoryLoad = Date.now();
            if (flightViewerActive) {
                flightViewerCharts[chartIdx] = { data, labels, unit, series };
                drawFlightChart(chartIdx);
            } else {
                chartLabels = labels;
                chartUnit = unit;
                chartSeriesCount = series;
                chartData = data;
                drawChart();
            }
        }
        break;
    }
}

function renderLoop() {
    if (latestBno && latestBno.ts !== lastRender.bno) {
        applyBno(latestBno);
        lastRender.bno = latestBno.ts;
    }
    if (latestCompass && latestCompass.ts !== lastRender.compass) {
        updateCompass(latestCompass);
        lastRender.compass = latestCompass.ts;
    }
    if (latestBmp && latestBmp.ts !== lastRender.bmp) {
        applyBmp585(latestBmp);
        lastRender.bmp = latestBmp.ts;
    }
    if (latestScd && latestScd.ts !== lastRender.scd) {
        applyScd40(latestScd);
        lastRender.scd = latestScd.ts;
    }
    if (latestSps && latestSps.ts !== lastRender.sps) {
        applySps30(latestSps);
        lastRender.sps = latestSps.ts;
    }
    if (latestGps && latestGps.ts !== lastRender.gps) {
        updateGPS(latestGps);
        lastRender.gps = latestGps.ts;
    }
    if (latestSys && latestSys.ts !== lastRender.sys) {
        applySystemStats(latestSys);
        lastRender.sys = latestSys.ts;
    }
    if (latestTime && latestTime.unix !== lastRender.time) {
        applyTimeWs(latestTime);
        lastRender.time = latestTime.unix;
    }
    requestAnimationFrame(renderLoop);
}

function quatNormalize(q) {
    const norm = Math.hypot(q.w, q.x, q.y, q.z);
    if (!norm) return { w: 1, x: 0, y: 0, z: 0 };
    return { w: q.w / norm, x: q.x / norm, y: q.y / norm, z: q.z / norm };
}

function quatConjugate(q) {
    return { w: q.w, x: -q.x, y: -q.y, z: -q.z };
}

function quatMultiply(a, b) {
    return {
        w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
}

function quatToMatrix(q) {
    const { w, x, y, z } = q;
    const xx = x * x, yy = y * y, zz = z * z;
    const xy = x * y, xz = x * z, yz = y * z;
    const wx = w * x, wy = w * y, wz = w * z;

    return [
        [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
        [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)]
    ];
}

function axisMapMatrix() {
    const axisVec = (axis, sign) => {
        switch (axis) {
        case 'x':
            return [sign, 0, 0];
        case 'y':
            return [0, sign, 0];
        case 'z':
            return [0, 0, sign];
        default:
            return [sign, 0, 0];
        }
    };

    const xRow = axisVec(imuAxisMap.xAxis, imuAxisMap.xSign);
    const yRow = axisVec(imuAxisMap.yAxis, imuAxisMap.ySign);
    const zRow = axisVec(imuAxisMap.zAxis, imuAxisMap.zSign);

    return [xRow, yRow, zRow];
}

function loadImuMapState() {
    try {
        const raw = localStorage.getItem('imuAxisMap');
        if (!raw) return { ...imuMapDefaults };
        const parsed = JSON.parse(raw);
        return {
            xAxis: parsed.xAxis || imuMapDefaults.xAxis,
            xSign: parsed.xSign === -1 ? -1 : 1,
            yAxis: parsed.yAxis || imuMapDefaults.yAxis,
            ySign: parsed.ySign === -1 ? -1 : 1,
            zAxis: parsed.zAxis || imuMapDefaults.zAxis,
            zSign: parsed.zSign === -1 ? -1 : 1
        };
    } catch (e) {
        return { ...imuMapDefaults };
    }
}

function saveImuMapState(state) {
    try {
        localStorage.setItem('imuAxisMap', JSON.stringify(state));
    } catch (e) {}
}

function applyImuMapState(state) {
    imuAxisMap.xAxis = state.xAxis;
    imuAxisMap.xSign = state.xSign;
    imuAxisMap.yAxis = state.yAxis;
    imuAxisMap.ySign = state.ySign;
    imuAxisMap.zAxis = state.zAxis;
    imuAxisMap.zSign = state.zSign;
}

function setImuZero() {
    if (lastQuatNorm) {
        zeroQuat = { ...lastQuatNorm };
    }
}

function initImuMappingControls() {
    const xAxis = $('imu-map-x-axis');
    const yAxis = $('imu-map-y-axis');
    const zAxis = $('imu-map-z-axis');
    const xInv = $('imu-map-x-invert');
    const yInv = $('imu-map-y-invert');
    const zInv = $('imu-map-z-invert');
    const applyBtn = $('imu-map-apply');
    const zeroBtn = $('imu-map-zero');
    const resetBtn = $('imu-map-reset');

    if (!xAxis || !yAxis || !zAxis || !xInv || !yInv || !zInv) {
        return;
    }

    const state = loadImuMapState();
    applyImuMapState(state);

    xAxis.value = state.xAxis;
    yAxis.value = state.yAxis;
    zAxis.value = state.zAxis;
    xInv.checked = state.xSign === -1;
    yInv.checked = state.ySign === -1;
    zInv.checked = state.zSign === -1;

    const readControls = () => ({
        xAxis: xAxis.value,
        xSign: xInv.checked ? -1 : 1,
        yAxis: yAxis.value,
        ySign: yInv.checked ? -1 : 1,
        zAxis: zAxis.value,
        zSign: zInv.checked ? -1 : 1
    });

    const apply = () => {
        const next = readControls();
        applyImuMapState(next);
        saveImuMapState(next);
        setImuZero();
    };

    xAxis.addEventListener('change', apply);
    yAxis.addEventListener('change', apply);
    zAxis.addEventListener('change', apply);
    xInv.addEventListener('change', apply);
    yInv.addEventListener('change', apply);
    zInv.addEventListener('change', apply);

    if (applyBtn) applyBtn.addEventListener('click', apply);
    if (zeroBtn) zeroBtn.addEventListener('click', setImuZero);
    if (resetBtn) {
        resetBtn.addEventListener('click', () => {
            applyImuMapState({ ...imuMapDefaults });
            saveImuMapState({ ...imuMapDefaults });
            xAxis.value = imuMapDefaults.xAxis;
            yAxis.value = imuMapDefaults.yAxis;
            zAxis.value = imuMapDefaults.zAxis;
            xInv.checked = imuMapDefaults.xSign === -1;
            yInv.checked = imuMapDefaults.ySign === -1;
            zInv.checked = imuMapDefaults.zSign === -1;
            setImuZero();
        });
    }
}

function matMul(a, b) {
    const out = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ];
    for (let r = 0; r < 3; r++) {
        for (let c = 0; c < 3; c++) {
            out[r][c] = a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c];
        }
    }
    return out;
}

function matTranspose(m) {
    return [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]]
    ];
}

function applyAxisMap(R) {
    const A = axisMapMatrix();
    const AT = matTranspose(A);
    return matMul(matMul(A, R), AT);
}

function matrixToEuler(R) {
    const clamp = (v, min, max) => Math.max(min, Math.min(max, v));
    const pitch = Math.asin(clamp(-R[2][0], -1, 1));
    let roll = 0;
    let yaw = 0;

    if (Math.abs(R[2][0]) < 0.9999) {
        roll = Math.atan2(R[2][1], R[2][2]);
        yaw = Math.atan2(R[1][0], R[0][0]);
    } else {
        roll = Math.atan2(-R[1][2], R[1][1]);
        yaw = 0;
    }

    return {
        roll: roll * radToDeg,
        pitch: pitch * radToDeg,
        yaw: ((yaw * radToDeg) % 360 + 360) % 360
    };
}

function matrixToCss(R) {
    return `matrix3d(${R[0][0]},${R[1][0]},${R[2][0]},0,` +
           `${R[0][1]},${R[1][1]},${R[2][1]},0,` +
           `${R[0][2]},${R[1][2]},${R[2][2]},0,` +
           `0,0,0,1)`;
}

function updateImuUI(q, accRad, accStatus, ts) {
    lastImuTs = ts;
    const qNorm = quatNormalize(q);
    lastQuatNorm = qNorm;
    if (autoZeroOnFirstSample && !zeroQuat) {
        zeroQuat = { ...qNorm };
    }

    const qRel = zeroQuat ? quatMultiply(quatConjugate(zeroQuat), qNorm) : qNorm;
    const R = applyAxisMap(quatToMatrix(qRel));
    const e = matrixToEuler(R);

    if ($('bno-quat')) $('bno-quat').textContent =
        `i:${fmt3(q.x)} j:${fmt3(q.y)} k:${fmt3(q.z)} w:${fmt3(q.w)}`;

    if ($('bno-euler')) $('bno-euler').textContent =
        `roll:${fmt3(e.roll)} pitch:${fmt3(e.pitch)} yaw:${fmt3(e.yaw)}`;
    if ($('bno-acc')) $('bno-acc').textContent =
        `${imuAccText[accStatus] || '--'} (${fmt3(accRad)})`;
    if ($('bno-status')) $('bno-status').textContent = accStatus >= 2 ? 'Tracking' : 'Calibrating';
    if ($('bno-ts')) $('bno-ts').textContent = ts ? `${(ts / 1000).toFixed(2)}s` : '--';

    const cube = $('imuCube');
    if (cube) {
        if (!cube.dataset.zeroHooked) {
            cube.dataset.zeroHooked = '1';
            cube.addEventListener('click', setImuZero);
        }
        cube.style.transform = matrixToCss(R);
    }

    const rig = $('imuRig');
    if (rig) {
        rig.style.transform = matrixToCss(R);
    }
}

function applySps30(d) {
    if (!d) return;
    const pm1 = d.pm1;
    const pm25 = d.pm25;
    const pm4 = d.pm4;
    const pm10 = d.pm10;
    const nc05 = d.nc05;
    const nc1 = d.nc1;
    const nc25 = d.nc25;
    const nc4 = d.nc4;
    const nc10 = d.nc10;
    const size = d.size;
    const ts = d.ts;

    // Dashboard
    if ($('pm1')) $('pm1').textContent = fmt(pm1);
    if ($('pm25')) $('pm25').textContent = fmt(pm25);
    if ($('pm4')) $('pm4').textContent = fmt(pm4);
    if ($('pm10')) $('pm10').textContent = fmt(pm10);

    // SPS30 page
    if ($('sps-pm1')) $('sps-pm1').textContent = fmt(pm1);
    if ($('sps-pm25')) $('sps-pm25').textContent = fmt(pm25);
    if ($('sps-pm4')) $('sps-pm4').textContent = fmt(pm4);
    if ($('sps-pm10')) $('sps-pm10').textContent = fmt(pm10);
    if ($('sps-nc05')) $('sps-nc05').textContent = fmt(nc05);
    if ($('sps-nc1')) $('sps-nc1').textContent = fmt(nc1);
    if ($('sps-nc25')) $('sps-nc25').textContent = fmt(nc25);
    if ($('sps-nc4')) $('sps-nc4').textContent = fmt(nc4);
    if ($('sps-nc10')) $('sps-nc10').textContent = fmt(nc10);
    if ($('sps-size')) $('sps-size').textContent = fmt(size);
    if ($('spsStatus')) $('spsStatus').textContent = 'Measuring';

    // Chart data is now loaded from flight recorder via loadFlightChart()
    // No direct SPS30 push needed - all sensor data goes through flight_data_collector
}

function applyBno(d) {
    if (!d) return;
    updateImuUI({ w: d.qw, x: d.qi, y: d.qj, z: d.qk }, d.acc, d.accStatus, d.ts);
}

function applyBmp585(d) {
    if (!d) return;
    lastBmpAlt = d.alt;
    const relAlt = bmpAltZero != null ? d.alt - bmpAltZero : null;
    if ($('bmp-temp')) $('bmp-temp').textContent = fmt4(d.temp);
    if ($('bmp-press')) $('bmp-press').textContent = fmt4(d.press_hpa);
    if ($('bmp-temp-big')) $('bmp-temp-big').textContent = fmt4(d.temp);
    if ($('bmp-press-big')) $('bmp-press-big').textContent = fmt4(d.press);
    if ($('bmp-press-hpa')) $('bmp-press-hpa').textContent = fmt4(d.press_hpa);
    if ($('bmp-alt')) $('bmp-alt').textContent = fmt4(d.alt);
    if ($('bmp-alt-rel')) $('bmp-alt-rel').textContent = relAlt != null ? fmt4(relAlt) : '--';
    if ($('bmp-alt-rel-big')) $('bmp-alt-rel-big').textContent = relAlt != null ? fmt4(relAlt) : '--';
    if ($('bmp-status')) $('bmp-status').textContent = 'OK';
    if ($('bmp-ts')) $('bmp-ts').textContent = d.ts ? `${(d.ts / 1000).toFixed(2)}s` : '--';
}

function applyScd40(d) {
    if (!d) return;
    if ($('co2')) $('co2').textContent = fmt(d.co2);
    if ($('scd-hum')) $('scd-hum').textContent = fmt(d.hum);
    if ($('scd-co2')) $('scd-co2').textContent = fmt(d.co2);
    if ($('scd-temp')) $('scd-temp').textContent = fmt(d.temp);
    if ($('scd-hum-big')) $('scd-hum-big').textContent = fmt(d.hum);
    if ($('scd-status')) $('scd-status').textContent = 'OK';
    if ($('scd-ts')) $('scd-ts').textContent = d.ts ? `${(d.ts / 1000).toFixed(2)}s` : '--';
}

function applySystemStats(d) {
    if (!d) return;
    const hasRssi = (d.flags & 0x01) !== 0;
    const hasTemp = (d.flags & 0x02) !== 0;
    const wifiRssi = hasRssi ? d.rssi : null;
    const cpuTemp = hasTemp ? d.temp : null;
    if ($('wifi-rssi')) $('wifi-rssi').textContent = wifiRssi != null ? wifiRssi : '--';
    if ($('wifi-quality')) $('wifi-quality').textContent = wifiQualityLabel(wifiRssi);
    if ($('cpu-temp')) $('cpu-temp').textContent = cpuTemp != null ? fmt(cpuTemp) : '--';
    // Uptime from system stats (ms since boot)
    if (d.ts != null && $('uptime')) {
        const secs = Math.floor(d.ts / 1000);
        const mins = Math.floor(secs / 60);
        const hrs = Math.floor(mins / 60);
        if (hrs > 0) {
            $('uptime').textContent = `${hrs}h ${mins % 60}m`;
        } else {
            $('uptime').textContent = `${mins}m ${secs % 60}s`;
        }
    }
}

function formatUnixWithOffset(unix, offsetMin) {
    if (!unix) return '--';
    const ms = (unix + offsetMin * 60) * 1000;
    const d = new Date(ms);
    const pad = n => String(n).padStart(2, '0');
    return `${d.getUTCFullYear()}-${pad(d.getUTCMonth() + 1)}-${pad(d.getUTCDate())}` +
           ` ${pad(d.getUTCHours())}:${pad(d.getUTCMinutes())}:${pad(d.getUTCSeconds())}`;
}

function applyTimeWs(d) {
    const synced = (d.flags & 0x01) !== 0;
    const timeStr = synced ? formatUnixWithOffset(d.unix, d.offsetMin) : '--';
    if ($('deviceTime')) $('deviceTime').textContent = timeStr;
    if ($('localTime')) {
        const timePart = timeStr.includes(' ') ? timeStr.split(' ')[1] : timeStr;
        $('localTime').textContent = timePart;
    }
    if ($('timeSource')) {
        const map = { 1: 'GPS', 2: 'NTP' };
        $('timeSource').textContent = synced ? (map[d.source] || 'SYNCED') : 'Not synced';
    }
}

// Toast
function toast(msg, type = 'info') {
    const t = $('toast');
    t.textContent = msg;
    t.className = 'toast show ' + type;
    setTimeout(() => t.className = 'toast', 3000);
}

// API helper
async function api(endpoint, method = 'GET', body = null) {
    const opts = { method };
    if (body) {
        opts.headers = { 'Content-Type': 'application/json' };
        opts.body = body;
    }
    const res = await fetch('/api/' + endpoint, opts);
    if (!res.ok) throw new Error(res.statusText);
    return res.json();
}

// ============================================================================
// Navigation
// ============================================================================

function toggleMenu() {
    const menu = $('sideMenu');
    const overlay = $('menuOverlay');
    const btn = $('menuBtn');
    const isOpen = menu.classList.toggle('open');
    overlay.classList.toggle('show', isOpen);
    btn.classList.toggle('open', isOpen);
}

function showPage(page) {
    currentPage = page;
    document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
    document.querySelectorAll('.menu-item').forEach(m => m.classList.remove('active'));
    $('page-' + page).classList.add('active');
    document.querySelector(`[data-page="${page}"]`).classList.add('active');

    // Close menu
    $('sideMenu').classList.remove('open');
    $('menuOverlay').classList.remove('show');
    $('menuBtn').classList.remove('open');

    // Page-specific init
    if (page === 'dashboard') {
        chartCanvasId = 'chart';
        flightViewerActive = false;
        resizeChart();
    }
    if (page === 'files') { loadFiles(); updatePsramInfo(); }
    if (page === 'system') { loadWifiStatus(); loadOtaInfo(); }
}

document.querySelectorAll('.menu-item').forEach(item => {
    item.addEventListener('click', e => {
        e.preventDefault();
        showPage(item.dataset.page);
    });
});

// ============================================================================
// Device Info
// ============================================================================

async function loadInfo() {
    try {
        const info = await api('info');
        $('serial').textContent = info.serial || '--';
        $('firmware').textContent = info.fw_major !== undefined ? `${info.fw_major}.${info.fw_minor}` : '--';
        if ($('spsSerial')) $('spsSerial').textContent = info.serial || '--';
        if ($('spsCleanInterval') && info.clean_interval !== undefined) {
            $('spsCleanInterval').textContent = info.clean_interval.toLocaleString();
        }
        const wifiRssi = typeof info.wifi_rssi === 'number' ? info.wifi_rssi : null;
        if ($('wifi-rssi')) $('wifi-rssi').textContent = wifiRssi != null ? wifiRssi : '--';
        if ($('wifi-quality')) $('wifi-quality').textContent = wifiQualityLabel(wifiRssi);
        const cpuTemp = typeof info.cpu_temp_c === 'number' ? info.cpu_temp_c : null;
        if ($('cpu-temp')) $('cpu-temp').textContent = cpuTemp != null ? fmt(cpuTemp) : '--';

        // Flight recorder PSRAM info (Files page)
        if ($('psramCount')) {
            $('psramCount').textContent = info.flight_records != null ? info.flight_records.toLocaleString() : '--';
        }
        if ($('psramCap')) {
            $('psramCap').textContent = info.flight_capacity != null ? info.flight_capacity.toLocaleString() : '--';
        }
    } catch (e) {}
}

// POSIX TZ: negative offset = east of UTC (opposite of common notation)
const tzStrings = {
    '0': 'UTC0',
    '1': 'CET-1CEST,M3.5.0,M10.5.0/3',      // UTC+1 / +2 DST
    '2': 'EET-2EEST,M3.5.0/3,M10.5.0/4',    // UTC+2 / +3 DST (Budapest)
    '3': 'MSK-3',                             // UTC+3 no DST
    '-5': 'EST5EDT,M3.2.0,M11.1.0',          // UTC-5 / -4 DST
    '-8': 'PST8PDT,M3.2.0,M11.1.0'           // UTC-8 / -7 DST
};

async function setTimezone(tz) {
    try {
        const tzStr = tzStrings[tz] || 'UTC0';
        await api('time/timezone', 'POST', JSON.stringify({tz: tzStr}));
        toast('Timezone set', 'success');
    } catch (e) { toast('Failed', 'error'); }
}

async function syncTimeFromBrowser() {
    try {
        const unix = Math.floor(Date.now() / 1000);
        await api(`time/sync?unix=${unix}`, 'POST');
        toast('Time synced', 'success');
    } catch (e) { toast('Failed', 'error'); }
}

// ============================================================================
// Sensor Controls
// ============================================================================

function toggleSensor(name, enabled) {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(`S,${name},${enabled ? 1 : 0}`);
}

function applySensorState(state) {
    if (!state) return;
    const setToggle = (id, on) => {
        const el = $(id);
        if (el) el.checked = !!on;
    };

    setToggle('toggle-gps', state.gps);
    setToggle('toggle-sps30', state.sps30);
    setToggle('toggle-compass', state.compass);
    setToggle('toggle-bno', state.bno);
    setToggle('toggle-bmp585', state.bmp585);
    setToggle('toggle-scd40', state.scd40);

    if ($('spsStatus')) $('spsStatus').textContent = state.sps30 ? 'Measuring' : 'Disabled';
    if ($('bno-status') && !state.bno) $('bno-status').textContent = 'Disabled';
    if ($('bmp-status') && !state.bmp585) $('bmp-status').textContent = 'Disabled';
    if ($('scd-status') && !state.scd40) $('scd-status').textContent = 'Disabled';
}

// ============================================================================
// SPS30 Controls
// ============================================================================

async function fanClean() {
    const btn = $('btnFan');
    btn.disabled = true;
    try {
        await api('fan-clean', 'POST');
        toast('Fan cleaning started', 'success');
        setTimeout(() => btn.disabled = false, 12000);
    } catch (e) { btn.disabled = false; toast('Failed', 'error'); }
}

async function resetSensor() {
    if (!confirm('Reset sensor?')) return;
    try {
        await api('reset', 'POST');
        toast('Sensor reset', 'success');
    } catch (e) { toast('Failed', 'error'); }
}

// ============================================================================
// Flight Data Chart
// ============================================================================

const chartColors = ['#58a6ff', '#f0883e', '#a371f7', '#f85149'];

function loadFlightChart() {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(`F,${chartType},${chartRange},300`);
    }
}

function setChartType(type) {
    chartType = parseInt(type);
    // Set default labels/unit immediately for better UX
    chartLabels = flightChartLabelsMap[chartType] || [];
    chartUnit = flightChartUnitsMap[chartType] || '';
    chartSeriesCount = chartLabels.length || 1;
    loadFlightChart();
}

function setRange(sec) {
    chartRange = sec;
    liveMode = (sec <= 300);
    document.querySelectorAll('.range-btn').forEach(b =>
        b.classList.toggle('active', parseInt(b.dataset.range) === sec));
    const live = $('liveIndicator');
    if (live) live.style.display = liveMode ? 'inline' : 'none';
    loadFlightChart();
}

function startFlightChartUpdates() {
    if (flightChartTimer) clearInterval(flightChartTimer);
    loadFlightChart();
    // Update chart every 2 seconds in live mode
    flightChartTimer = setInterval(() => {
        if (liveMode && currentPage === 'dashboard') {
            loadFlightChart();
        }
    }, 2000);
}

function drawChart() {
    const canvas = $(chartCanvasId);
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    const pad = { l: 50, r: 15, t: 30, b: 25 };
    const plotW = w - pad.l - pad.r;
    const plotH = h - pad.t - pad.b;
    const data = (chartData.length > 1 && chartData[0][0] > chartData[chartData.length - 1][0])
        ? [...chartData].reverse()
        : chartData;

    ctx.fillStyle = '#0d1117';
    ctx.fillRect(0, 0, w, h);

    if (data.length < 2) {
        ctx.fillStyle = '#484f58';
        ctx.font = '12px system-ui';
        ctx.textAlign = 'center';
        ctx.fillText('Waiting for flight data...', w / 2, h / 2);
        return;
    }

    // Get all values for Y scale (skip timestamp at index 0)
    const allVals = data.flatMap(d => d.slice(1).filter((v, i) => i < chartSeriesCount));
    let minY = Math.min(...allVals);
    let maxY = Math.max(...allVals);
    const rangeY = maxY - minY || 1;

    // Adjust Y scale with some padding
    const yPad = rangeY * 0.1;
    minY = minY - yPad;
    maxY = maxY + yPad;

    // Round to nice values
    const step = (maxY - minY) / 4;
    minY = Math.floor(minY / step) * step;
    maxY = Math.ceil(maxY / step) * step;
    if (maxY - minY < 0.01) maxY = minY + 1;

    // Y grid and labels
    ctx.strokeStyle = '#21262d';
    ctx.lineWidth = 1;
    ctx.font = '10px system-ui';
    ctx.textAlign = 'right';
    ctx.fillStyle = '#484f58';

    for (let i = 0; i <= 4; i++) {
        const y = pad.t + (plotH / 4) * i;
        const val = maxY - ((maxY - minY) / 4) * i;
        ctx.beginPath();
        ctx.moveTo(pad.l, y);
        ctx.lineTo(w - pad.r, y);
        ctx.stroke();
        // Format based on range
        const valStr = Math.abs(val) >= 100 ? val.toFixed(0) : val.toFixed(1);
        ctx.fillText(valStr, pad.l - 6, y + 3);
    }

    // X axis - timestamps
    const minT = data[0][0];
    const maxT = data[data.length - 1][0];
    // Unix timestamps in seconds (> 1577836800 = 2020-01-01)
    const isUnixTime = minT > 1577836800;

    ctx.textAlign = 'center';
    for (let i = 0; i <= 4; i++) {
        const x = pad.l + (plotW / 4) * i;
        const t = minT + ((maxT - minT) / 4) * i;
        let label;
        if (isUnixTime) {
            const d = new Date(t * 1000);
            label = d.getHours() + ':' + d.getMinutes().toString().padStart(2, '0');
        } else {
            const sec = Math.floor(t);
            const min = Math.floor(sec / 60) % 60;
            const hr = Math.floor(sec / 3600);
            label = `${hr}:${min.toString().padStart(2, '0')}`;
        }
        ctx.fillText(label, x, h - 6);
    }

    // Draw data series
    for (let s = 0; s < chartSeriesCount; s++) {
        ctx.strokeStyle = chartColors[s % chartColors.length];
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        data.forEach((d, i) => {
            const val = d[s + 1];  // Skip timestamp at index 0
            if (val == null) return;
            const x = pad.l + (i / (data.length - 1)) * plotW;
            const y = pad.t + plotH - ((val - minY) / (maxY - minY)) * plotH;
            i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
        });
        ctx.stroke();
    }

    // Legend
    ctx.font = '10px system-ui';
    chartLabels.forEach((label, i) => {
        if (i >= chartSeriesCount || !label) return;
        const x = pad.l + 8 + i * 60;
        ctx.fillStyle = chartColors[i % chartColors.length];
        ctx.fillRect(x, 6, 10, 10);
        ctx.fillStyle = '#c9d1d9';
        ctx.textAlign = 'left';
        ctx.fillText(label, x + 13, 14);
    });

    // Unit label
    ctx.fillStyle = '#666';
    ctx.textAlign = 'right';
    ctx.fillText(chartUnit, w - pad.r, 14);
}

function drawChartCustom(canvasId, data, labels, unit, series) {
    const canvas = $(canvasId);
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    const pad = { l: 50, r: 15, t: 30, b: 25 };
    const plotW = w - pad.l - pad.r;
    const plotH = h - pad.t - pad.b;
    const points = (data.length > 1 && data[0][0] > data[data.length - 1][0])
        ? [...data].reverse()
        : data;

    ctx.fillStyle = '#0d1117';
    ctx.fillRect(0, 0, w, h);

    if (points.length < 2) {
        ctx.fillStyle = '#484f58';
        ctx.font = '12px system-ui';
        ctx.textAlign = 'center';
        ctx.fillText('Waiting for flight data...', w / 2, h / 2);
        return;
    }

    const allVals = points.flatMap(d => d.slice(1).filter((v, i) => i < series));
    let minY = Math.min(...allVals);
    let maxY = Math.max(...allVals);
    const rangeY = maxY - minY || 1;

    const yPad = rangeY * 0.1;
    minY = minY - yPad;
    maxY = maxY + yPad;

    const step = (maxY - minY) / 4;
    minY = Math.floor(minY / step) * step;
    maxY = Math.ceil(maxY / step) * step;
    if (maxY - minY < 0.01) maxY = minY + 1;

    ctx.strokeStyle = '#21262d';
    ctx.lineWidth = 1;
    ctx.font = '10px system-ui';
    ctx.textAlign = 'right';
    ctx.fillStyle = '#484f58';

    for (let i = 0; i <= 4; i++) {
        const y = pad.t + (plotH / 4) * i;
        const val = maxY - ((maxY - minY) / 4) * i;
        ctx.beginPath();
        ctx.moveTo(pad.l, y);
        ctx.lineTo(w - pad.r, y);
        ctx.stroke();
        ctx.fillText(val.toFixed(1), pad.l - 6, y + 3);
    }

    const minT = points[0][0];
    const maxT = points[points.length - 1][0];
    const isUnixTime = minT > 1577836800;

    ctx.textAlign = 'center';
    for (let i = 0; i <= 4; i++) {
        const x = pad.l + (plotW / 4) * i;
        const t = minT + ((maxT - minT) / 4) * i;
        let label;
        if (isUnixTime) {
            const d = new Date(t * 1000);
            label = d.getHours() + ':' + d.getMinutes().toString().padStart(2, '0');
        } else {
            const sec = Math.floor(t);
            const min = Math.floor(sec / 60) % 60;
            const hr = Math.floor(sec / 3600);
            label = `${hr}:${min.toString().padStart(2, '0')}`;
        }
        ctx.fillStyle = '#484f58';
        ctx.fillText(label, x, h - 8);
    }

    for (let s = 0; s < series; s++) {
        ctx.strokeStyle = chartColors[s % chartColors.length];
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        points.forEach((d, i) => {
            const val = d[s + 1];
            if (val == null) return;
            const x = pad.l + (i / (points.length - 1)) * plotW;
            const y = pad.t + plotH - ((val - minY) / (maxY - minY)) * plotH;
            i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
        });
        ctx.stroke();
    }

    labels.forEach((label, i) => {
        if (i >= series || !label) return;
        const x = pad.l + 8 + i * 60;
        ctx.fillStyle = chartColors[i % chartColors.length];
        ctx.fillRect(x, 6, 10, 10);
        ctx.fillStyle = '#c9d1d9';
        ctx.textAlign = 'left';
        ctx.fillText(label, x + 13, 14);
    });

    ctx.fillStyle = '#666';
    ctx.textAlign = 'right';
    ctx.fillText(unit, w - pad.r, 14);
}

function drawFlightChart(chartIdx) {
    const canvasId = flightViewerCanvasByType[chartIdx];
    if (!canvasId) return;
    const chart = flightViewerCharts[chartIdx] || {};
    drawChartCustom(
        canvasId,
        chart.data || [],
        chart.labels || flightChartLabelsMap[chartIdx] || [],
        chart.unit || flightChartUnitsMap[chartIdx] || '',
        chart.series || (flightChartLabelsMap[chartIdx] ? flightChartLabelsMap[chartIdx].length : 1)
    );
}

function resizeChartCanvas(id, height) {
    const canvas = $(id);
    if (!canvas || !canvas.parentElement) return;
    canvas.width = Math.min(canvas.parentElement.clientWidth - 24, 1000);
    canvas.height = height;
    drawChart();
}

function resizeChart() {
    resizeChartCanvas('chart', 220);
}

function resizeFlightViewerCharts() {
    for (let i = 0; i < flightViewerCanvasByType.length; i++) {
        const canvasId = flightViewerCanvasByType[i];
        const canvas = $(canvasId);
        if (!canvas || !canvas.parentElement) continue;
        canvas.width = Math.min(canvas.parentElement.clientWidth - 24, 1000);
        canvas.height = 220;
        const chart = flightViewerCharts[i] || {};
        drawChartCustom(
            canvasId,
            chart.data || [],
            chart.labels || flightChartLabelsMap[i] || [],
            chart.unit || flightChartUnitsMap[i] || '',
            chart.series || (flightChartLabelsMap[i] ? flightChartLabelsMap[i].length : 1)
        );
    }
}

// ============================================================================
// Recording
// ============================================================================

let recActive = false;
let recStartTime = 0;
let recTimer = null;

function formatDuration(sec) {
    const h = Math.floor(sec / 3600);
    const m = Math.floor((sec % 3600) / 60);
    const s = sec % 60;
    if (h > 0) return `${h}:${m.toString().padStart(2,'0')}:${s.toString().padStart(2,'0')}`;
    return `${m}:${s.toString().padStart(2,'0')}`;
}

function updateRecTimer() {
    const info = $('recInfo');
    if (!info || !recActive) return;
    const elapsed = Math.floor((Date.now() - recStartTime) / 1000);
    info.textContent = formatDuration(elapsed);
}

async function checkRecording() {
    try {
        const data = await api('recording/status');
        recActive = data.active;
        const btn = $('btnRec');
        const info = $('recInfo');
        if (!btn) return;

        if (data.active) {
            btn.textContent = 'Stop';
            btn.classList.add('recording');
            recStartTime = Date.now() - (data.duration_sec || 0) * 1000;
            info.textContent = formatDuration(data.duration_sec || 0);
            info.style.display = 'inline';
            if (!recTimer) recTimer = setInterval(updateRecTimer, 1000);
        } else {
            btn.textContent = 'Record';
            btn.classList.remove('recording');
            info.style.display = 'none';
            if (recTimer) { clearInterval(recTimer); recTimer = null; }
        }
    } catch (e) {}
}

async function toggleRec() {
    try {
        if (recActive) {
            const res = await api('recording/stop', 'POST');
            toast(`Saved: ${res.records || 0} records`, 'success');
        } else {
            await api('recording/start', 'POST');
            toast('Recording started', 'success');
        }
        checkRecording();
    } catch (e) { toast('Failed', 'error'); }
}


// ============================================================================
// WebSocket
// ============================================================================

function resetActivityTimer() {
    lastActivityTime = Date.now();
    if (activityTimer) clearTimeout(activityTimer);
    activityTimer = setTimeout(checkConnectionHealth, CONNECTION_TIMEOUT_MS);
}

function checkConnectionHealth() {
    const elapsed = Date.now() - lastActivityTime;
    if (elapsed >= CONNECTION_TIMEOUT_MS && ws && ws.readyState === WebSocket.OPEN) {
        console.warn('Connection timeout - no data for', Math.round(elapsed/1000), 'seconds, reconnecting...');
        ws.close();
    }
}

function connect() {
    // Prevent multiple connections
    if (ws && (ws.readyState === WebSocket.CONNECTING || ws.readyState === WebSocket.OPEN)) {
        return;
    }
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
    if (activityTimer) { clearTimeout(activityTimer); activityTimer = null; }

    ws = new WebSocket('ws://' + location.host + '/ws');
    ws.binaryType = 'arraybuffer';

    ws.onopen = () => {
        $('status').textContent = 'Connected';
        $('status').className = 'status online';
        resetActivityTimer();
        loadInfo();
        checkRecording();
        ws.send('Q');
    };

    ws.onclose = (ev) => {
        $('status').textContent = 'Offline';
        $('status').className = 'status offline';
        if (activityTimer) { clearTimeout(activityTimer); activityTimer = null; }
        if (ev && ev.code !== 1000) {
            console.warn('WebSocket closed', { code: ev.code, reason: ev.reason, clean: ev.wasClean });
        }
        ws = null;
        if (reconnect) reconnectTimer = setTimeout(connect, 2000);
    };

    ws.onerror = (err) => {
        console.warn('WebSocket error', err);
        // Don't null ws here - let onclose handle cleanup
    };

    ws.onmessage = (e) => {
        resetActivityTimer();  // Any message resets the timeout

        if (typeof e.data === 'string') {
            try {
                const msg = JSON.parse(e.data);
                if (msg.type === 'sensors') {
                    applySensorState(msg);
                }
            } catch (err) {}
            return;
        }

        if (e.data instanceof ArrayBuffer && e.data.byteLength > 0) {
            handleBinaryMessage(e.data);
        }
    };
}

// ============================================================================
// GPS & Compass
// ============================================================================

function updateGPS(g) {
    if (!g) return;
    const fixTypes = ['No fix', 'Dead reckoning', '2D', '3D', 'GNSS+DR', 'Time'];

    // Dashboard
    if ($('gpsFix')) $('gpsFix').textContent = fixTypes[g.fix] || 'Unknown';
    if ($('gpsSat')) $('gpsSat').textContent = g.sat;
    if ($('gpsAlt')) $('gpsAlt').textContent = g.alt.toFixed(0);
    if ($('gpsSpeed')) $('gpsSpeed').textContent = g.speed.toFixed(1);

    if (g.fix >= 2 && g.lat !== 0) {
        if ($('gpsLat')) $('gpsLat').textContent = g.lat.toFixed(6);
        if ($('gpsLon')) $('gpsLon').textContent = g.lon.toFixed(6);
        if ($('gpsLink')) $('gpsLink').href = `https://www.google.com/maps?q=${g.lat},${g.lon}`;
        if ($('gpsTime')) $('gpsTime').textContent = g.time.replace('T', ' ');
    } else {
        if ($('gpsLat')) $('gpsLat').textContent = '--';
        if ($('gpsLon')) $('gpsLon').textContent = '--';
        if ($('gpsLink')) $('gpsLink').href = '#';
        if ($('gpsTime')) $('gpsTime').textContent = '--';
    }

    // GPS page - full details
    if ($('gps-lat')) $('gps-lat').textContent = g.lat.toFixed(7);
    if ($('gps-lon')) $('gps-lon').textContent = g.lon.toFixed(7);
    if ($('gps-alt')) $('gps-alt').textContent = g.alt.toFixed(1);
    if ($('gps-height')) $('gps-height').textContent = g.height ? g.height.toFixed(1) : '--';
    if ($('gps-hacc')) $('gps-hacc').textContent = (g.hacc / 1000).toFixed(2);
    if ($('gps-vacc')) $('gps-vacc').textContent = (g.vacc / 1000).toFixed(2);
    if ($('gps-sacc')) $('gps-sacc').textContent = g.sacc ? (g.sacc / 1000).toFixed(2) : '--';
    if ($('gps-pdop')) $('gps-pdop').textContent = g.pdop ? (g.pdop / 100).toFixed(1) : '--';
    if ($('gps-fixtype')) $('gps-fixtype').textContent = fixTypes[g.fix] || 'Unknown';
    if ($('gps-sat')) $('gps-sat').textContent = g.sat;
    if ($('gps-valid')) $('gps-valid').textContent = g.fix >= 2 ? 'Yes' : 'No';
    if ($('gps-flags')) $('gps-flags').textContent = g.flags ? `0x${g.flags.toString(16).toUpperCase()}` : '--';
    if ($('gps-speed')) $('gps-speed').textContent = g.speed.toFixed(1);
    if ($('gps-heading')) $('gps-heading').textContent = g.heading.toFixed(1);
    if ($('gps-veln')) $('gps-veln').textContent = g.veln ? (g.veln / 1000).toFixed(2) : '--';
    if ($('gps-vele')) $('gps-vele').textContent = g.vele ? (g.vele / 1000).toFixed(2) : '--';
    if ($('gps-date')) $('gps-date').textContent = g.time ? g.time.split('T')[0] : '--';
    if ($('gps-time')) $('gps-time').textContent = g.time ? g.time.split('T')[1] : '--';
    if ($('gps-tacc')) $('gps-tacc').textContent = g.tacc || '--';
    if ($('gps-itow')) $('gps-itow').textContent = g.itow || '--';
    if ($('gpsMapLink')) $('gpsMapLink').href = g.fix >= 2 ? `https://www.google.com/maps?q=${g.lat},${g.lon}` : '#';
}

function updateCompass(m) {
    if (!m) return;

    // Dashboard
    if ($('compassHeading')) {
        $('compassHeading').textContent = m.heading.toFixed(0) + '\u00B0';
        drawCompass($('compassCanvas'), m.heading, 50);
    }

    // GPS page
    if ($('compass-heading')) {
        $('compass-heading').textContent = m.heading.toFixed(0) + '\u00B0';
        drawCompass($('compassCanvasBig'), m.heading, 120);
    }
    if ($('mag-x')) $('mag-x').textContent = m.x;
    if ($('mag-y')) $('mag-y').textContent = m.y;
    if ($('mag-z')) $('mag-z').textContent = m.z;
}

function drawCompass(canvas, heading, size) {
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = size, h = size;
    const cx = w / 2, cy = h / 2, r = w / 2 - 4;

    ctx.fillStyle = '#0d1117';
    ctx.fillRect(0, 0, w, h);

    ctx.strokeStyle = '#30363d';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();

    const fontSize = size < 80 ? 8 : 11;
    ctx.font = `${fontSize}px system-ui`;
    ctx.fillStyle = '#484f58';
    ctx.textAlign = 'center';
    ctx.fillText('N', cx, fontSize + 2);
    ctx.fillText('S', cx, h - 3);
    ctx.fillText('E', w - 4, cy + 3);
    ctx.fillText('W', 6, cy + 3);

    const rad = (heading - 90) * Math.PI / 180;
    ctx.strokeStyle = '#f85149';
    ctx.lineWidth = size < 80 ? 2 : 3;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(rad) * (r - 6), cy + Math.sin(rad) * (r - 6));
    ctx.stroke();

    ctx.fillStyle = '#3fb950';
    ctx.beginPath();
    ctx.arc(cx, cy, size < 80 ? 3 : 4, 0, Math.PI * 2);
    ctx.fill();
}

// ============================================================================
// Files
// ============================================================================

async function loadFiles() {
    const list = $('fileList');
    const info = $('storageInfo');
    if (!list) return;

    try {
        const data = await api('fs-list');
        const files = data.files || [];

        if (files.length === 0) {
            list.innerHTML = '<div class="file-item"><span class="file-name">No files yet</span></div>';
        } else {
            list.innerHTML = files.map(f => {
                const sizeKB = (f.size / 1024).toFixed(1);
                const isFlightRec = f.name.endsWith('.bin') && f.name.startsWith('flight_');
                // flight_*.bin = 12 byte header + 64 byte records
                const entries = isFlightRec ? Math.max(0, Math.floor((f.size - 12) / 64)) : 0;
                const meta = isFlightRec ? `${sizeKB} KB · ~${entries} entries` : `${sizeKB} KB`;
                const downloadUrl = `/api/fs-download?file=/${encodeURIComponent(f.name)}&raw=1`;
                return `<div class="file-item">
                    <div>
                        <span class="file-name">${f.name}</span>
                        <span class="file-size">${meta}</span>
                    </div>
                    <div class="file-actions">
                        ${isFlightRec ? `<button class="btn-sm primary" onclick="loadToChart('${f.name}')">Chart</button>` : ''}
                        <button class="btn-sm" onclick="hexDump('${f.name}')">Hex</button>
                        <a href="${downloadUrl}" class="btn-sm">Download</a>
                        <button class="btn-sm danger" onclick="deleteFile('${f.name}')">Delete</button>
                    </div>
                </div>`;
            }).join('');
        }

        // Storage info
        if (info && data.total && data.used !== undefined) {
            const totalKB = Math.round(data.total / 1024);
            const usedKB = Math.round(data.used / 1024);
            const freeKB = totalKB - usedKB;
            info.innerHTML = `<span>Used: <b>${usedKB} KB</b></span><span>Free: <b>${freeKB} KB</b></span><span>Total: <b>${totalKB} KB</b></span>`;
        }

        const flightSection = $('flightViewerSection');
        if (flightSection && !flightViewerActive) {
            flightSection.classList.remove('active');
        }

    } catch (e) {
        list.innerHTML = '<div class="file-item"><span class="file-name">Error loading files</span></div>';
    }
}

async function deleteFile(name) {
    if (!confirm(`Delete ${name}?`)) return;
    try {
        await api(`fs-delete?file=/${encodeURIComponent(name)}`, 'POST');
        toast('Deleted', 'success');
        loadFiles();
    } catch (e) { toast('Failed', 'error'); }
}

function loadToChart(name) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        liveMode = false;  // Stop live updates
        flightViewerActive = true;
        for (let i = 0; i < flightViewerCanvasByType.length; i++) {
            delete flightViewerCharts[i];
        }
        const flightSection = $('flightViewerSection');
        if (flightSection) flightSection.classList.add('active');
        if ($('flightInfoFile')) $('flightInfoFile').textContent = name;
        resizeFlightViewerCharts();
        for (let i = 0; i < flightViewerCanvasByType.length; i++) {
            ws.send(`L,${i},${name}`);
        }
        showPage('files');
        toast('Loading ' + name);
    }
}

async function hexDump(name) {
    try {
        const res = await fetch('/api/fs-download?file=/' + encodeURIComponent(name) + '&raw=1');
        const buffer = await res.arrayBuffer();
        const bytes = new Uint8Array(buffer);
        renderHex(bytes, name);
    } catch (e) {
        toast('Hexdump failed', 'error');
    }
}

function renderHex(bytes, label, note = '') {
    const limit = Math.min(bytes.length, 1024);
    const lines = [];
    for (let i = 0; i < limit; i += 16) {
        const slice = bytes.slice(i, i + 16);
        const hex = Array.from(slice).map(b => b.toString(16).padStart(2, '0')).join(' ');
        const ascii = Array.from(slice).map(b => (b >= 32 && b <= 126) ? String.fromCharCode(b) : '.').join('');
        lines.push(i.toString(16).padStart(4, '0') + '  ' + hex.padEnd(47, ' ') + '  ' + ascii);
    }
    $('hexTitle').textContent = `${label} · ${bytes.length} bytes (showing first ${limit})${note}`;
    $('hexBox').textContent = lines.join('\n');
    $('hexSection').classList.add('active');
}

function clearHex() {
    $('hexSection').classList.remove('active');
    $('hexTitle').textContent = '--';
    $('hexBox').textContent = '--';
}

function downloadFile(content, filename, type) {
    const blob = new Blob([content], { type });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url; a.download = filename; a.click();
    URL.revokeObjectURL(url);
}

// ============================================================================
// PSRAM Download & Bin File Viewer
// ============================================================================

function downloadPsramBin() {
    window.open('/api/flight-export?format=bin', '_blank');
}

function handleDragOver(e) {
    e.preventDefault();
    e.stopPropagation();
    e.currentTarget.classList.add('drag-over');
}

function handleDragLeave(e) {
    e.preventDefault();
    e.stopPropagation();
    e.currentTarget.classList.remove('drag-over');
}

function handleFileDrop(e) {
    e.preventDefault();
    e.stopPropagation();
    e.currentTarget.classList.remove('drag-over');
    const files = e.dataTransfer.files;
    if (files.length > 0) {
        loadBinFile(files[0]);
    }
}

function handleFileSelect(e) {
    const files = e.target.files;
    if (files.length > 0) {
        loadBinFile(files[0]);
    }
}

function loadBinFile(file) {
    if (!file.name.endsWith('.bin')) {
        toast('Please select a .bin file', 'error');
        return;
    }

    const reader = new FileReader();
    reader.onload = (e) => {
        const buffer = e.target.result;
        parseBinAndShowCharts(buffer, file.name);
    };
    reader.onerror = () => toast('Failed to read file', 'error');
    reader.readAsArrayBuffer(file);
}

function parseBinAndShowCharts(buffer, filename) {
    const data = new DataView(buffer);

    // Check magic header 0x464C5452 ("FLTR" as little-endian uint32)
    const magic = data.getUint32(0, true);
    if (magic !== 0x464C5452) {
        toast('Invalid file format (expected FLTR header)', 'error');
        return;
    }

    // Header: magic(4) + version(1) + record_size(1) + reserved(2) + start_timestamp(4) = 12 bytes
    const version = data.getUint8(4);
    const recordSize = data.getUint8(5);
    // reserved: 2 bytes at offset 6
    const startTimestamp = data.getUint32(8, true);

    if (recordSize !== 64) {
        toast(`Unexpected record size: ${recordSize}`, 'error');
        return;
    }

    // Calculate record count from file size
    const headerSize = 12;
    const recordCount = Math.floor((buffer.byteLength - headerSize) / recordSize);
    const records = [];

    for (let i = 0; i < recordCount; i++) {
        const offset = headerSize + i * 64;
        if (offset + 64 > buffer.byteLength) break;

        records.push({
            timestamp: data.getUint32(offset, true),
            lat: data.getInt32(offset + 4, true) / 1e7,
            lon: data.getInt32(offset + 8, true) / 1e7,
            alt_mm: data.getInt32(offset + 12, true),
            vel_down: data.getInt16(offset + 16, true),
            fix_type: data.getUint8(offset + 18),
            sats: data.getUint8(offset + 19),
            pdop: data.getUint16(offset + 20, true) / 100,
            gps_valid: data.getUint8(offset + 22),
            qi: data.getInt16(offset + 23, true) / 16384,
            qj: data.getInt16(offset + 25, true) / 16384,
            qk: data.getInt16(offset + 27, true) / 16384,
            qw: data.getInt16(offset + 29, true) / 16384,
            imu_accuracy: data.getUint8(offset + 31),
            imu_valid: data.getUint8(offset + 32),
            mag_x: data.getInt16(offset + 33, true),
            mag_y: data.getInt16(offset + 35, true),
            mag_z: data.getInt16(offset + 37, true),
            heading: data.getInt16(offset + 39, true) / 10,
            pressure_pa: data.getInt32(offset + 41, true),
            bmp_temp: data.getInt16(offset + 45, true) / 100,
            baro_alt: data.getInt16(offset + 47, true) / 10,
            co2: data.getUint16(offset + 49, true),
            scd_temp: data.getInt16(offset + 51, true) / 100,
            humidity: data.getUint16(offset + 53, true) / 100,
            pm1: data.getUint16(offset + 55, true) / 10,
            pm25: data.getUint16(offset + 57, true) / 10,
            pm4: data.getUint16(offset + 59, true) / 10,
            pm10: data.getUint16(offset + 61, true) / 10
        });
    }

    if (records.length === 0) {
        toast('No records found in file', 'error');
        return;
    }

    // Calculate duration
    const startTs = records[0].timestamp;
    const endTs = records[records.length - 1].timestamp;
    const durationSec = endTs - startTs;
    const hours = Math.floor(durationSec / 3600);
    const mins = Math.floor((durationSec % 3600) / 60);
    const secs = durationSec % 60;
    const durationStr = hours > 0 ? `${hours}h ${mins}m ${secs}s` : `${mins}m ${secs}s`;

    // Update info
    if ($('flightInfoFile')) $('flightInfoFile').textContent = filename;
    if ($('flightInfoRecords')) $('flightInfoRecords').textContent = records.length.toLocaleString();
    if ($('flightInfoDuration')) $('flightInfoDuration').textContent = durationStr;

    // Show viewer section
    const section = $('flightViewerSection');
    if (section) section.style.display = 'block';

    // Prepare chart data
    const timestamps = records.map(r => r.timestamp);

    // Downsample if too many points
    const maxPoints = 2000;
    let step = 1;
    if (records.length > maxPoints) {
        step = Math.ceil(records.length / maxPoints);
    }

    const sampledRecords = [];
    const sampledTs = [];
    for (let i = 0; i < records.length; i += step) {
        sampledRecords.push(records[i]);
        sampledTs.push(timestamps[i]);
    }

    // Render all charts
    renderFlightChart('flightChartPm', sampledTs, [
        sampledRecords.map(r => r.pm1),
        sampledRecords.map(r => r.pm25),
        sampledRecords.map(r => r.pm4),
        sampledRecords.map(r => r.pm10)
    ], ['PM1', 'PM2.5', 'PM4', 'PM10'], 'µg/m³');

    renderFlightChart('flightChartCo2', sampledTs, [
        sampledRecords.map(r => r.co2)
    ], ['CO2'], 'ppm');

    renderFlightChart('flightChartPress', sampledTs, [
        sampledRecords.map(r => r.pressure_pa / 100)
    ], ['Pressure'], 'hPa');

    renderFlightChart('flightChartAlt', sampledTs, [
        sampledRecords.map(r => r.baro_alt),
        sampledRecords.map(r => r.alt_mm / 1000)
    ], ['Baro', 'GPS'], 'm');

    renderFlightChart('flightChartTemp', sampledTs, [
        sampledRecords.map(r => r.bmp_temp),
        sampledRecords.map(r => r.scd_temp)
    ], ['BMP', 'SCD'], '°C');

    renderFlightChart('flightChartHum', sampledTs, [
        sampledRecords.map(r => r.humidity)
    ], ['Humidity'], '%');

    renderFlightChart('flightChartVel', sampledTs, [
        sampledRecords.map(r => r.vel_down / 1000)
    ], ['Velocity'], 'm/s');

    renderFlightChart('flightChartHead', sampledTs, [
        sampledRecords.map(r => r.heading)
    ], ['Heading'], '°');

    // IMU - calculate Euler angles from quaternion
    renderFlightChart('flightChartImu', sampledTs, [
        sampledRecords.map(r => {
            // Roll from quaternion
            const sinr = 2 * (r.qw * r.qi + r.qj * r.qk);
            const cosr = 1 - 2 * (r.qi * r.qi + r.qj * r.qj);
            return Math.atan2(sinr, cosr) * 57.2957795;
        }),
        sampledRecords.map(r => {
            // Pitch from quaternion
            const sinp = 2 * (r.qw * r.qj - r.qk * r.qi);
            return Math.abs(sinp) >= 1 ? Math.sign(sinp) * 90 : Math.asin(sinp) * 57.2957795;
        }),
        sampledRecords.map(r => {
            // Yaw from quaternion
            const siny = 2 * (r.qw * r.qk + r.qi * r.qj);
            const cosy = 1 - 2 * (r.qj * r.qj + r.qk * r.qk);
            return Math.atan2(siny, cosy) * 57.2957795;
        })
    ], ['Roll', 'Pitch', 'Yaw'], '°');

    toast(`Loaded ${records.length} records`, 'success');
}

function renderFlightChart(canvasId, timestamps, series, labels, unit) {
    const canvas = $(canvasId);
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    const w = canvas.width = canvas.parentElement.clientWidth;
    const h = canvas.height = 150;

    ctx.fillStyle = '#1a1a2e';
    ctx.fillRect(0, 0, w, h);

    if (!timestamps || timestamps.length === 0) return;

    const colors = ['#4a9eff', '#4caf50', '#ff9800', '#e91e63', '#9c27b0'];
    const padding = { left: 50, right: 20, top: 10, bottom: 25 };
    const chartW = w - padding.left - padding.right;
    const chartH = h - padding.top - padding.bottom;

    // Find min/max across all series
    let minVal = Infinity, maxVal = -Infinity;
    for (const s of series) {
        for (const v of s) {
            if (v !== null && v !== undefined && isFinite(v)) {
                if (v < minVal) minVal = v;
                if (v > maxVal) maxVal = v;
            }
        }
    }
    if (!isFinite(minVal)) minVal = 0;
    if (!isFinite(maxVal)) maxVal = 1;
    if (minVal === maxVal) { minVal -= 1; maxVal += 1; }
    const range = maxVal - minVal;

    // Draw grid
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 0.5;
    for (let i = 0; i <= 4; i++) {
        const y = padding.top + (chartH * i / 4);
        ctx.beginPath();
        ctx.moveTo(padding.left, y);
        ctx.lineTo(w - padding.right, y);
        ctx.stroke();
    }

    // Draw Y axis labels
    ctx.fillStyle = '#888';
    ctx.font = '10px monospace';
    ctx.textAlign = 'right';
    for (let i = 0; i <= 4; i++) {
        const val = maxVal - (range * i / 4);
        const y = padding.top + (chartH * i / 4);
        ctx.fillText(val.toFixed(1), padding.left - 5, y + 3);
    }

    // Draw series
    const minTs = timestamps[0];
    const maxTs = timestamps[timestamps.length - 1];
    const tsRange = maxTs - minTs || 1;

    for (let si = 0; si < series.length; si++) {
        const data = series[si];
        ctx.strokeStyle = colors[si % colors.length];
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        let started = false;
        for (let i = 0; i < data.length; i++) {
            const v = data[i];
            if (v === null || v === undefined || !isFinite(v)) continue;
            const x = padding.left + ((timestamps[i] - minTs) / tsRange) * chartW;
            const y = padding.top + chartH - ((v - minVal) / range) * chartH;
            if (!started) {
                ctx.moveTo(x, y);
                started = true;
            } else {
                ctx.lineTo(x, y);
            }
        }
        ctx.stroke();
    }

    // Legend
    ctx.font = '10px sans-serif';
    let legendX = padding.left;
    for (let si = 0; si < labels.length; si++) {
        ctx.fillStyle = colors[si % colors.length];
        ctx.fillRect(legendX, h - 12, 10, 10);
        ctx.fillStyle = '#aaa';
        ctx.textAlign = 'left';
        ctx.fillText(labels[si], legendX + 14, h - 3);
        legendX += ctx.measureText(labels[si]).width + 30;
    }

    // Unit
    ctx.fillStyle = '#666';
    ctx.textAlign = 'right';
    ctx.fillText(unit, w - padding.right, padding.top + 10);
}

// Update PSRAM info on Files page
async function updatePsramInfo() {
    try {
        const info = await api('info');
        if ($('psramCount')) {
            $('psramCount').textContent = info.flight_records != null ? info.flight_records.toLocaleString() : '--';
        }
        if ($('psramCap')) {
            $('psramCap').textContent = info.flight_capacity != null ? info.flight_capacity.toLocaleString() : '--';
        }
    } catch (e) {}
}

// ============================================================================
// WiFi
// ============================================================================

async function loadWifiStatus() {
    try {
        const res = await fetch('/api/wifi/status');
        const data = await res.json();
        const el = $('wifiStatus');
        if (data.mode === 'STA') {
            el.className = 'wifi-status connected';
            el.textContent = `Connected to: ${data.ssid} (${data.ip})`;
        } else {
            el.className = 'wifi-status ap';
            el.textContent = `AP Mode: ${data.ap_ssid} (192.168.4.1)`;
        }
    } catch(e) {}
}

async function wifiConnect() {
    const ssid = $('wifiSsid').value.trim();
    const pass = $('wifiPass').value;
    if (!ssid) { showToast('Enter SSID'); return; }
    try {
        const res = await fetch('/api/wifi/connect', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({ssid, password: pass})
        });
        const data = await res.json();
        if (data.ok) {
            showToast('Saved! Rebooting...');
        } else {
            showToast('Error: ' + (data.error || 'Failed'));
        }
    } catch(e) { showToast('Error: ' + e.message); }
}

async function wifiClear() {
    if (!confirm('Clear saved WiFi credentials?')) return;
    try {
        await fetch('/api/wifi/clear', {method: 'POST'});
        showToast('Credentials cleared');
    } catch(e) { showToast('Error'); }
}

// OTA
// ============================================================================

// Firmware OTA
function otaFwFileSelected() {
    const file = $('otaFwFile').files[0];
    $('otaFwBtn').disabled = !file;
}

function otaUploadFw() {
    otaUpload('otaFwFile', 'otaFwBtn', 'otaFwProgress', 'otaFwBar', 'otaFwPercent', 'otaFwStatus', '/api/ota');
}

// Web UI Files Upload
function otaUiFileSelected() {
    const files = $('otaUiFiles').files;
    $('otaUiBtn').disabled = !files.length;
}

async function otaUploadUi() {
    const files = Array.from($('otaUiFiles').files || []);
    if (!files.length) return;

    const btn = $('otaUiBtn');
    const progress = $('otaUiProgress');
    const bar = $('otaUiBar');
    const percent = $('otaUiPercent');
    const status = $('otaUiStatus');

    btn.disabled = true;
    progress.style.display = 'flex';

    for (let i = 0; i < files.length; i++) {
        const f = files[i];
        status.textContent = `Uploading ${f.name} (${i + 1}/${files.length})...`;
        status.style.color = 'var(--text)';

        try {
            const res = await fetch(`/api/fs-upload?file=/${encodeURIComponent(f.name)}`, {
                method: 'POST',
                body: f
            });
            if (!res.ok) {
                const msg = await res.text();
                throw new Error(msg || res.statusText);
            }
        } catch (err) {
            status.textContent = `Failed: ${err.message}`;
            status.style.color = 'var(--red)';
            btn.disabled = false;
            return;
        }

        const pct = Math.round(((i + 1) / files.length) * 100);
        bar.style.width = pct + '%';
        percent.textContent = pct + '%';
    }

    status.textContent = `Uploaded ${files.length} file(s) successfully`;
    status.style.color = 'var(--green)';
    btn.disabled = false;
    toast('UI files uploaded', 'success');
}

// LittleFS Image OTA
function otaFsFileSelected() {
    const file = $('otaFsFile').files[0];
    $('otaFsBtn').disabled = !file;
}

function otaUploadFs() {
    otaUpload('otaFsFile', 'otaFsBtn', 'otaFsProgress', 'otaFsBar', 'otaFsPercent', 'otaFsStatus', '/api/fs-ota');
}

// Generic OTA upload function
function otaUpload(fileId, btnId, progressId, barId, percentId, statusId, endpoint) {
    const file = $(fileId).files[0];
    if (!file) return;

    const btn = $(btnId);
    const progress = $(progressId);
    const bar = $(barId);
    const percent = $(percentId);
    const status = $(statusId);

    btn.disabled = true;
    progress.style.display = 'flex';
    status.textContent = 'Uploading...';
    status.style.color = 'var(--text)';

    const xhr = new XMLHttpRequest();
    xhr.open('POST', endpoint);

    xhr.upload.onprogress = (e) => {
        if (e.lengthComputable) {
            const pct = Math.round((e.loaded / e.total) * 100);
            bar.style.width = pct + '%';
            percent.textContent = pct + '%';
        }
    };

    xhr.onload = () => {
        if (xhr.status === 200) {
            status.textContent = 'Success! Rebooting...';
            status.style.color = 'var(--green)';
            setTimeout(() => location.reload(), 5000);
        } else {
            status.textContent = 'Failed: ' + xhr.responseText;
            status.style.color = 'var(--red)';
            btn.disabled = false;
        }
    };

    xhr.onerror = () => {
        status.textContent = 'Upload failed';
        status.style.color = 'var(--red)';
        btn.disabled = false;
    };

    xhr.send(file);
}

async function loadOtaInfo() {
    try {
        const info = await api('info');
        if ($('otaFwVersion')) $('otaFwVersion').textContent = info.fw_major !== undefined ? `${info.fw_major}.${info.fw_minor}` : '--';
        if ($('otaPartition')) $('otaPartition').textContent = info.partition || '--';
    } catch (e) {}

    // Load filesystem info
    try {
        const fsInfo = await api('fs-list');
        if ($('otaFsInfo') && fsInfo.total) {
            const usedKB = Math.round(fsInfo.used / 1024);
            const totalKB = Math.round(fsInfo.total / 1024);
            $('otaFsInfo').textContent = `${usedKB}/${totalKB} KB`;
        }
    } catch (e) {}
}

// ============================================================================
// Init
// ============================================================================

window.addEventListener('resize', () => {
    resizeChart();
    if (flightViewerActive) resizeFlightViewerCharts();
});
window.addEventListener('beforeunload', () => {
    reconnect = false;
    if (ws) ws.close();
});

resizeChart();
connect();
initImuMappingControls();
initBmpAltZeroControls();
requestAnimationFrame(renderLoop);
startFlightChartUpdates();

function togglePass() {
    const p = $('wifiPass');
    p.type = p.type === 'password' ? 'text' : 'password';
}
