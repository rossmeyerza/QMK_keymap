const HID = require('node-hid');
const si = require('systeminformation');

// QMK Raw HID protocol constants
const USAGE_PAGE = 0xFF60;
const USAGE      = 0x0061;
const MSG_TYPE_STATS = 0x01;
const PACKET_SIZE = 32;
const POLL_INTERVAL_MS = 2000;

// Cache last good GPU values (nvidia-smi sometimes returns null)
let lastGpuPercent = 0;
let lastGpuTempC = 0;

// Find the QMK keyboard's Raw HID interface
function findKeyboard() {
    const devices = HID.devices();
    const rawHidDevices = devices.filter(d =>
        d.usagePage === USAGE_PAGE && d.usage === USAGE
    );

    if (rawHidDevices.length === 0) {
        console.error('No QMK Raw HID device found.');
        console.error('Make sure your keyboard is plugged in and RAW_ENABLE = yes in rules.mk');
        console.error('\nAll HID devices with usage page 0xFF60:');
        devices.filter(d => d.usagePage === USAGE_PAGE).forEach(d =>
            console.log(`  VID: 0x${d.vendorId.toString(16)} PID: 0x${d.productId.toString(16)} - ${d.product}`)
        );
        return null;
    }

    if (rawHidDevices.length > 1) {
        console.log('Multiple Raw HID devices found:');
        rawHidDevices.forEach((d, i) =>
            console.log(`  [${i}] VID: 0x${d.vendorId.toString(16)} PID: 0x${d.productId.toString(16)} - ${d.product}`)
        );
        console.log('Using the first one.');
    }

    const target = rawHidDevices[0];
    console.log(`Found: ${target.product || 'Unknown'} (VID: 0x${target.vendorId.toString(16)}, PID: 0x${target.productId.toString(16)})`);
    return new HID.HID(target.path);
}

// Gather system stats and pack into HID packet
async function gatherAndSend(device) {
    const [cpu, mem, cpuTemp, graphics] = await Promise.all([
        si.currentLoad(),
        si.mem(),
        si.cpuTemperature(),
        si.graphics()
    ]);

    const cpuPercent = Math.min(Math.round(cpu.currentLoad), 100);
    const ramPercent = Math.min(Math.round((mem.used / mem.total) * 100), 100);
    const cpuTempC  = Math.round(cpuTemp.main || 0);

    // Try to get GPU stats from the first controller, cache last good values
    if (graphics.controllers && graphics.controllers.length > 0) {
        const gpu = graphics.controllers[0];
        if (gpu.utilizationGpu !== undefined && gpu.utilizationGpu !== null && gpu.utilizationGpu >= 0) {
            lastGpuPercent = Math.min(Math.round(gpu.utilizationGpu), 100);
        }
        if (gpu.temperatureGpu !== undefined && gpu.temperatureGpu !== null) {
            lastGpuTempC = Math.round(gpu.temperatureGpu);
        }
    }
    const gpuPercent = lastGpuPercent;
    const gpuTempC = lastGpuTempC;

    // Build packet: [msg_type, cpu%, ram%, cpu_temp, reserved, gpu%, gpu_temp, ...zeros]
    const buf = Buffer.alloc(PACKET_SIZE + 1, 0); // +1 for report ID prefix
    buf[0] = 0x00;              // Report ID (required by node-hid on some platforms)
    buf[1] = MSG_TYPE_STATS;    // msg_type
    buf[2] = cpuPercent;        // cpu %
    buf[3] = ramPercent;        // ram %
    buf[4] = cpuTempC;          // cpu temp
    buf[5] = 0x00;              // reserved
    buf[6] = gpuPercent;        // gpu %
    buf[7] = gpuTempC;          // gpu temp

    device.write(buf);

    console.log(`CPU: ${cpuPercent}% ${cpuTempC}C | RAM: ${ramPercent}% | GPU: ${gpuPercent === 0xFF ? 'N/A' : gpuPercent + '%'} ${gpuTempC}C`);
}

// Main loop with auto-reconnection
let device = null;

async function tick() {
    try {
        if (!device) {
            device = findKeyboard();
            if (!device) return;

            device.on('error', (err) => {
                console.error('Device error:', err.message);
                device = null;
            });

            device.on('data', (data) => {
                // Handle responses from keyboard (e.g. heartbeat echo)
            });
        }
        await gatherAndSend(device);
    } catch (err) {
        console.error('Error:', err.message);
        if (device) {
            try { device.close(); } catch (_) {}
        }
        device = null;
    }
}

console.log('QMK System Stats - Starting...');
console.log(`Polling every ${POLL_INTERVAL_MS / 1000}s\n`);

// Initial tick, then repeat
tick();
const interval = setInterval(tick, POLL_INTERVAL_MS);

// Graceful shutdown
process.on('SIGINT', () => {
    console.log('\nShutting down...');
    clearInterval(interval);
    if (device) {
        try { device.close(); } catch (_) {}
    }
    process.exit(0);
});
