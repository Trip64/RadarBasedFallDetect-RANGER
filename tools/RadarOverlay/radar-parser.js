(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  root.RangerRadarParser = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  function numberMap(text) {
    const values = {};
    const pattern = /([A-Za-z_][A-Za-z0-9_]*)\s*[:=]\s*(-?(?:\d+(?:\.\d*)?|\.\d+))/g;
    let match;
    while ((match = pattern.exec(text)) !== null) {
      values[match[1].toLowerCase()] = Number(match[2]);
    }
    return values;
  }

  function parseCsvNumbers(text) {
    return text.split(",").map((part) => Number(part.trim()));
  }

  function parseLine(rawLine) {
    const line = String(rawLine || "").trim();
    if (!line) return { kind: "empty" };

    // Ranger 2: T1: x=-782mm y=1713mm spd=-16cm/s res=50mm VALID
    // Tolerates TARGET 1, optional units/resolution, and INVALID/--- states.
    const target = line.match(
      /\bT(?:ARGET)?\s*(\d+)\s*:?\s*x\s*=\s*(-?\d+)\s*(?:mm)?\s+y\s*=\s*(-?\d+)\s*(?:mm)?\s+(?:spd|speed)\s*=\s*(-?\d+)\s*(?:cm\/s)?(?:\s+res\s*=\s*(\d+)\s*(?:mm)?)?.*?\b(VALID|INVALID|---)\b/i
    );
    if (target) {
      return {
        kind: "target",
        id: Number(target[1]),
        xMm: Number(target[2]),
        yMm: Number(target[3]),
        speedCms: Number(target[4]),
        resolutionMm: target[5] === undefined ? null : Number(target[5]),
        valid: target[6].toUpperCase() === "VALID",
        raw: line,
      };
    }

    // Compact optional debug format: RADAR,slot,x_mm,y_mm,speed_cm_s,res_mm,valid
    const radarCsv = line.match(/^RADAR\s*,\s*(\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)(?:\s*,\s*(\d+))?(?:\s*,\s*(0|1))?\s*$/i);
    if (radarCsv) {
      return {
        kind: "target",
        id: Number(radarCsv[1]),
        xMm: Number(radarCsv[2]),
        yMm: Number(radarCsv[3]),
        speedCms: Number(radarCsv[4]),
        resolutionMm: radarCsv[5] === undefined ? null : Number(radarCsv[5]),
        valid: radarCsv[6] === undefined || radarCsv[6] === "1",
        raw: line,
      };
    }

    // Ranger 2 telemetry is parsed by key, so extra/reordered fields are safe.
    if (/^TELEMETRY\s*:/i.test(line)) {
      return { kind: "telemetry", values: numberMap(line), raw: line };
    }

    if (/^IMU\s*:/i.test(line)) {
      return { kind: "imu", values: numberMap(line), raw: line };
    }

    // Fusion/Base debug CSV. Raw IMU is 4096 LSB/g and 65.5 LSB/dps.
    const imuCsv = line.match(/(?:^|(?:RX>|\[BASE\])\s*)I\s*,\s*(.+)$/i);
    if (imuCsv) {
      const p = parseCsvNumbers(imuCsv[1]);
      if (p.length >= 6 && p.slice(0, 6).every(Number.isFinite)) {
        return {
          kind: "imu-raw",
          ax: p[0] / 4096,
          ay: p[1] / 4096,
          az: p[2] / 4096,
          gx: p[3] / 65.5,
          gy: p[4] / 65.5,
          gz: p[5] / 65.5,
          sequence: Number.isFinite(p[6]) ? p[6] : null,
          raw: line,
        };
      }
      return { kind: "malformed", expected: "IMU CSV", raw: line };
    }

    const envCsv = line.match(/(?:^|(?:RX>|\[BASE\])\s*)E\s*,\s*(.+)$/i);
    if (envCsv) {
      const p = parseCsvNumbers(envCsv[1]);
      if (p.length >= 6 && p.slice(0, 6).every(Number.isFinite)) {
        return {
          kind: "environment",
          pressurePa: p[0],
          ir: p[1],
          red: p[2],
          battery: p[3],
          sos: p[4] !== 0,
          sequence: p[5],
          raw: line,
        };
      }
      return { kind: "malformed", expected: "ENV CSV", raw: line };
    }

    if (line.includes("STAT,W_OK") || line.includes("STAT,BASE_LINK_OK")) {
      return { kind: "link", online: true, raw: line };
    }
    if (line.includes("STAT,W_LOST") || line.includes("STAT,LINK_LOST")) {
      return { kind: "link", online: false, raw: line };
    }

    if (/^\[STATS\]/.test(line)) {
      return { kind: "stats", values: numberMap(line), raw: line };
    }

    const mlPossible = line.match(/\[ML\].*FALL POSSIBLE.*score=(\d+)%.*conf=(\d+)%/i);
    if (mlPossible) {
      return {
        kind: "ml-event",
        state: "possible",
        fallPercent: Number(mlPossible[1]),
        confidencePercent: Number(mlPossible[2]),
        raw: line,
      };
    }
    if (/\[ML\].*FALL CONFIRMED/i.test(line)) {
      return { kind: "ml-event", state: "confirmed", raw: line };
    }
    if (/\[ML\].*False Alarm/i.test(line)) {
      return { kind: "ml-event", state: "false-alarm", raw: line };
    }

    const version = line.match(/(?:RANGER|---\s*RANGER)\s+(.+?)(?:\s*---)?$/i);
    if (version) return { kind: "version", value: version[1].trim(), raw: line };

    return { kind: "unknown", raw: line };
  }

  return { parseLine, numberMap };
});
