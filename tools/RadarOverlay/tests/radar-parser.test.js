"use strict";

const assert = require("node:assert/strict");
const { parseLine } = require("../radar-parser.js");

assert.deepEqual(parseLine("T1: x=-782mm y=1713mm spd=-16cm/s res=50mm VALID"), {
  kind: "target", id: 1, xMm: -782, yMm: 1713, speedCms: -16,
  resolutionMm: 50, valid: true,
  raw: "T1: x=-782mm y=1713mm spd=-16cm/s res=50mm VALID",
});

assert.equal(parseLine("T2: x=0mm y=0mm spd=0cm/s res=0mm INVALID").valid, false);
assert.equal(parseLine("RADAR,3,1200,2450,8,50,1").id, 3);

const telemetry = parseLine("TELEMETRY: radar=0.50 ax=0.12 az=1.01 ay=-0.03 mlConf=99 fusion=0.85 mlFall=94 gx=1.2 gy=-0.5 gz=0.3");
assert.equal(telemetry.kind, "telemetry");
assert.equal(telemetry.values.mlfall, 94);
assert.equal(telemetry.values.ax, 0.12);

const imu = parseLine("RX> I,4096,-2048,4096,655,-131,0,42");
assert.equal(imu.kind, "imu-raw");
assert.equal(imu.ax, 1);
assert.equal(imu.ay, -0.5);
assert.equal(imu.sequence, 42);

const env = parseLine("[BASE] E,100802,1,2,87,1,44");
assert.equal(env.kind, "environment");
assert.equal(env.battery, 87);
assert.equal(env.sos, true);

assert.deepEqual(parseLine("STAT,W_LOST"), { kind: "link", online: false, raw: "STAT,W_LOST" });
assert.equal(parseLine("[ML] FALL POSSIBLE: score=91% conf=96%").state, "possible");
assert.equal(parseLine("garbage").kind, "unknown");

console.log("Radar parser tests passed");
