const canvas = document.getElementById("plannerCanvas");
const ctx = canvas ? canvas.getContext("2d") : null;

const modeReadout = document.getElementById("mode-readout");
const waypointReadout = document.getElementById("waypoint-readout");
const hazardReadout = document.getElementById("hazard-readout");
const distanceReadout = document.getElementById("distance-readout");

const perceptionState = document.getElementById("perception-state");
const planningState = document.getElementById("planning-state");
const controlState = document.getElementById("control-state");

const waypoints = [
    { x: 0.0, y: 0.0 },
    { x: 0.0, y: 1.0 },
    { x: 0.0, y: 2.0 },
    { x: -1.0, y: 2.0 },
    { x: -1.99228, y: 2.12403 },
    { x: -1.74705, y: 3.0935 },
    { x: -1.50181, y: 4.06296 },
    { x: -1.25658, y: 5.03243 },
    { x: -1.01135, y: 6.00189 },
    { x: -0.766116, y: 6.97136 },
    { x: -0.520883, y: 7.94082 },
    { x: -0.275651, y: 8.91028 },
    { x: -0.0304182, y: 9.87975 },
    { x: 0.0, y: 10.0 }
];

const obstacle = {
    x: 0.0,
    y: 3.0,
    radius: 0.45
};

let currentWaypoint = 0;
let progress = 0;
let lastTimestamp = null;
let missionComplete = false;

function resizeCanvas() {
    if (!canvas || !ctx) return;

    const rect = canvas.getBoundingClientRect();
    const scale = window.devicePixelRatio || 1;

    canvas.width = Math.max(1, Math.floor(rect.width * scale));
    canvas.height = Math.max(1, Math.floor(rect.height * scale));

    ctx.setTransform(scale, 0, 0, scale, 0, 0);
}

function worldToCanvas(point, width, height) {
    const minX = -3.0;
    const maxX = 1.5;
    const minY = -0.5;
    const maxY = 10.5;

    const padding = 55;

    const usableWidth = width - padding * 2;
    const usableHeight = height - padding * 2;

    const x =
        padding +
        ((point.x - minX) / (maxX - minX)) *
        usableWidth;

    const y =
        height -
        padding -
        ((point.y - minY) / (maxY - minY)) *
        usableHeight;

    return { x, y };
}

function interpolate(a, b, t) {
    return {
        x: a.x + (b.x - a.x) * t,
        y: a.y + (b.y - a.y) * t
    };
}

function distance(a, b) {
    const dx = a.x - b.x;
    const dy = a.y - b.y;

    return Math.sqrt(dx * dx + dy * dy);
}

function drawAxes(width, height) {
    ctx.save();

    ctx.strokeStyle = "rgba(124, 141, 165, 0.22)";
    ctx.lineWidth = 1;

    const origin = worldToCanvas({ x: 0, y: 0 }, width, height);
    const top = worldToCanvas({ x: 0, y: 10 }, width, height);

    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(top.x, top.y);
    ctx.stroke();

    ctx.fillStyle = "rgba(124, 141, 165, 0.75)";
    ctx.font = "12px Courier New";

    for (let y = 0; y <= 10; y += 1) {
        const p = worldToCanvas({ x: 0, y }, width, height);

        ctx.fillText(
            y.toString(),
            p.x + 10,
            p.y + 4
        );
    }

    ctx.restore();
}

function drawPlannedCenterline(width, height) {
    ctx.save();

    ctx.strokeStyle = "rgba(56, 189, 248, 0.28)";
    ctx.lineWidth = 1;
    ctx.setLineDash([6, 6]);

    const start = worldToCanvas({ x: 0, y: 0 }, width, height);
    const target = worldToCanvas({ x: 0, y: 10 }, width, height);

    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    ctx.lineTo(target.x, target.y);
    ctx.stroke();

    ctx.setLineDash([]);
    ctx.restore();
}

function drawObstacle(width, height) {
    const p = worldToCanvas(obstacle, width, height);

    ctx.save();

    ctx.fillStyle = "rgba(248, 113, 113, 0.18)";
    ctx.strokeStyle = "#f87171";
    ctx.lineWidth = 2;

    ctx.beginPath();
    ctx.arc(p.x, p.y, 24, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = "#f87171";
    ctx.font = "12px Courier New";

    ctx.fillText(
        "RADAR OBSTACLE",
        p.x + 32,
        p.y - 6
    );

    ctx.fillText(
        "(0, 3)",
        p.x + 32,
        p.y + 12
    );

    ctx.restore();
}

function drawTrajectory(width, height) {
    ctx.save();

    ctx.strokeStyle = "#4ade80";
    ctx.lineWidth = 2.5;
    ctx.shadowColor = "rgba(74, 222, 128, 0.35)";
    ctx.shadowBlur = 7;

    ctx.beginPath();

    waypoints.forEach((point, index) => {
        const p = worldToCanvas(point, width, height);

        if (index === 0) {
            ctx.moveTo(p.x, p.y);
        } else {
            ctx.lineTo(p.x, p.y);
        }
    });

    ctx.stroke();
    ctx.restore();

    ctx.save();

    waypoints.forEach((point, index) => {
        const p = worldToCanvas(point, width, height);

        ctx.fillStyle =
            index === 0
                ? "#38bdf8"
                : index === waypoints.length - 1
                ? "#facc15"
                : "#4ade80";

        ctx.beginPath();
        ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
        ctx.fill();

        if (
            index === 0 ||
            index === 3 ||
            index === 4 ||
            index === waypoints.length - 1
        ) {
            ctx.fillStyle = "rgba(248,250,252,0.78)";
            ctx.font = "11px Courier New";

            ctx.fillText(
                `WP${index}`,
                p.x + 8,
                p.y - 8
            );
        }
    });

    ctx.restore();
}

function drawVehicle(position, width, height) {
    const p = worldToCanvas(position, width, height);

    ctx.save();

    ctx.translate(p.x, p.y);

    ctx.fillStyle = "#a78bfa";
    ctx.shadowColor = "rgba(167, 139, 250, 0.55)";
    ctx.shadowBlur = 10;

    ctx.beginPath();
    ctx.moveTo(0, -12);
    ctx.lineTo(9, 10);
    ctx.lineTo(0, 6);
    ctx.lineTo(-9, 10);
    ctx.closePath();
    ctx.fill();

    ctx.restore();
}

function updateTelemetry(vehiclePosition) {
    const obstacleDistance =
        distance(vehiclePosition, obstacle);

    if (waypointReadout) {
        waypointReadout.textContent =
            `${currentWaypoint} / ${waypoints.length - 1}`;
    }

    if (distanceReadout) {
        distanceReadout.textContent =
            obstacleDistance.toFixed(2);
    }

    const hazardActive =
        obstacleDistance < 1.15 &&
        vehiclePosition.y < 4.0;

    if (hazardReadout) {
        hazardReadout.textContent =
            hazardActive ? "DETECTED" : "CLEAR";

        hazardReadout.style.color =
            hazardActive ? "#f87171" : "#4ade80";
    }

    if (modeReadout) {
        if (missionComplete) {
            modeReadout.textContent = "MISSION COMPLETE";
        } else if (hazardActive) {
            modeReadout.textContent = "EVASIVE REPLAN";
        } else if (currentWaypoint < 3) {
            modeReadout.textContent = "NOMINAL TRACKING";
        } else {
            modeReadout.textContent = "PATH EXECUTION";
        }
    }

    if (perceptionState) {
        perceptionState.textContent = "ONLINE";
    }

    if (planningState) {
        planningState.textContent =
            missionComplete
                ? "COMPLETE"
                : hazardActive
                ? "REPLANNING"
                : "ACTIVE";
    }

    if (controlState) {
        controlState.textContent =
            missionComplete
                ? "COMPLETE"
                : currentWaypoint > 0
                ? "EXECUTING"
                : "STANDBY";
    }
}

function advanceSimulation(deltaSeconds) {
    if (missionComplete) return;

    progress += deltaSeconds * 0.65;

    if (progress >= 1) {
        progress = 0;
        currentWaypoint += 1;

        if (currentWaypoint >= waypoints.length - 1) {
            currentWaypoint = waypoints.length - 1;
            progress = 0;
            missionComplete = true;
        }
    }
}

function render(timestamp) {
    if (!canvas || !ctx) return;

    if (lastTimestamp === null) {
        lastTimestamp = timestamp;
    }

    const deltaSeconds =
        Math.min(
            (timestamp - lastTimestamp) / 1000,
            0.05
        );

    lastTimestamp = timestamp;

    advanceSimulation(deltaSeconds);

    const width = canvas.clientWidth;
    const height = canvas.clientHeight;

    ctx.clearRect(0, 0, width, height);

    drawAxes(width, height);
    drawPlannedCenterline(width, height);
    drawObstacle(width, height);
    drawTrajectory(width, height);

    const current =
        waypoints[currentWaypoint];

    const next =
        waypoints[
            Math.min(
                currentWaypoint + 1,
                waypoints.length - 1
            )
        ];

    const vehiclePosition =
        missionComplete
            ? current
            : interpolate(
                current,
                next,
                progress
            );

    drawVehicle(
        vehiclePosition,
        width,
        height
    );

    updateTelemetry(
        vehiclePosition
    );

    requestAnimationFrame(render);
}

if (canvas && ctx) {
    resizeCanvas();

    window.addEventListener(
        "resize",
        resizeCanvas
    );

    requestAnimationFrame(render);
}