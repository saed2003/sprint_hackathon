// Street View Robot — presentation deck (v2)
// ONE unified DARK theme · minimal text · heavy on visuals.
const pptxgen = require("pptxgenjs");
const React = require("react");
const ReactDOMServer = require("react-dom/server");
const sharp = require("sharp");
const fa = require("react-icons/fa");
const path = require("path");

const BASE = "C:/Users/saeda/Documents/university/4th_year/sprint/sprint_hackathon";
const IMG = {
  ring:  BASE + "/src/new_point_cloud/pointcloud_360_preview.png",
  irL:   BASE + "/src/depth_map/test_images/ir_left.png",
  irR:   BASE + "/src/depth_map/test_images/ir_right.png",
  depth: BASE + "/src/depth_map/result_ref/depth_color.png",
  scan:  (n) => BASE + `/src/captures/scan_20260602_174649/shot_0${n}/ir_left.png`,
  orbit: (n) => BASE + `/test_code/object_scan/captures/orbit_20260603_102236/shot_0${n}/color.png`,
};
const OUT = path.join(__dirname, "Street_View_Robot.pptx");

// ---- palette (dark) --------------------------------------------------------
const C = {
  bg:    "0E1A2B", // deep navy — every slide
  panel: "16263B", // card
  panel2:"1C3047", // raised card / hover
  line:  "2A4361", // border on dark
  matte: "081320", // near-black behind images so colour pops
  white: "FFFFFF",
  textL: "E8EFF7", // body on dark
  muted: "93A3B8", // muted on dark
  cyan:  "22D3EE", // primary accent
  cyanD: "0E9CC0",
  coral: "FB7256", // secondary accent
  coralD:"E2563B",
  amber: "F5B642",
  green: "34D399",
  violet:"A78BFA",
};
const F = { head: "Trebuchet MS", body: "Calibri", mono: "Consolas" };
const W = 13.33, H = 7.5;

async function icon(Comp, hex, size = 320) {
  const color = hex.startsWith("#") ? hex : "#" + hex;
  const svg = ReactDOMServer.renderToStaticMarkup(React.createElement(Comp, { color, size: String(size) }));
  const png = await sharp(Buffer.from(svg)).png().toBuffer();
  return "image/png;base64," + png.toString("base64");
}
async function arOf(p) { const m = await sharp(p).metadata(); return m.width / m.height; }

async function main() {
  // icons -----------------------------------------------------------------
  const I = {};
  const need = {
    robot: [fa.FaRobot, C.white], chip: [fa.FaMicrochip, C.white], camera: [fa.FaCamera, C.white],
    wave: [fa.FaWaveSquare, C.cyan], road: [fa.FaRoad, C.cyan], gamepad: [fa.FaGamepad, C.white],
    route: [fa.FaRoute, C.white], cubes: [fa.FaCubes, C.white], cube: [fa.FaCube, C.white],
    bolt: [fa.FaBolt, C.bg], graph: [fa.FaProjectDiagram, C.white], check: [fa.FaCheck, C.white],
    arrow: [fa.FaArrowRight, C.cyan], cross: [fa.FaCrosshairs, C.white], ruler: [fa.FaRulerCombined, C.white],
    sync: [fa.FaSyncAlt, C.bg], print: [fa.FaPrint, C.white], tools: [fa.FaTools, C.white],
    mesh: [fa.FaDrawPolygon, C.white], cad: [fa.FaVectorSquare, C.white], play: [fa.FaPlay, C.cyan],
    film: [fa.FaFilm, C.muted], image: [fa.FaImage, C.muted], map: [fa.FaMapMarkedAlt, C.white],
    layer: [fa.FaLayerGroup, C.white], target: [fa.FaBullseye, C.white], segment: [fa.FaObjectGroup, C.white],
    eye: [fa.FaEye, C.white], search: [fa.FaSearchPlus, C.white], dot: [fa.FaCircle, C.cyan],
  };
  for (const [k, [Comp, col]] of Object.entries(need)) I[k] = await icon(Comp, col);

  // image aspect ratios ---------------------------------------------------
  const AR = {
    ring: await arOf(IMG.ring), irL: await arOf(IMG.irL), irR: await arOf(IMG.irR),
    depth: await arOf(IMG.depth), scan: await arOf(IMG.scan(0)), orbit: await arOf(IMG.orbit(0)),
  };

  const pres = new pptxgen();
  pres.layout = "LAYOUT_WIDE";
  pres.author = "Street View Robot team";
  pres.title = "Street View Robot";

  const sh = () => ({ type: "outer", color: "05101C", blur: 10, offset: 4, angle: 90, opacity: 0.45 });

  // ---- helpers ----------------------------------------------------------
  function bg(slide) { slide.background = { color: C.bg }; }
  function head(slide, kicker, title, accent = C.cyan) {
    slide.addShape(pres.shapes.RECTANGLE, { x: 0.7, y: 0.58, w: 0.09, h: 0.86, fill: { color: accent } });
    slide.addText(kicker, { x: 0.95, y: 0.58, w: 11.6, h: 0.3, fontFace: F.body, fontSize: 12.5, bold: true, color: accent, charSpacing: 3, margin: 0 });
    slide.addText(title, { x: 0.95, y: 0.9, w: 11.9, h: 0.64, fontFace: F.head, fontSize: 29, bold: true, color: C.white, margin: 0 });
  }
  function footer(slide, n) {
    slide.addText("STREET VIEW ROBOT", { x: 0.7, y: 7.04, w: 6, h: 0.34, fontFace: F.body, fontSize: 9, color: "5C6E84", charSpacing: 2, valign: "middle", margin: 0 });
    slide.addText(String(n), { x: W - 1.25, y: 7.04, w: 0.5, h: 0.34, fontFace: F.body, fontSize: 10, color: "5C6E84", align: "right", valign: "middle", margin: 0 });
    slide.addShape(pres.shapes.RECTANGLE, { x: W - 0.64, y: 7.14, w: 0.12, h: 0.12, fill: { color: C.cyan } });
  }
  function card(slide, x, y, w, h, fill = C.panel, opts = {}) {
    slide.addShape(pres.shapes.ROUNDED_RECTANGLE, {
      x, y, w, h, rectRadius: 0.07, fill: { color: fill },
      line: opts.border ? { color: opts.border, width: opts.borderW || 1 } : { color: fill, width: 0 },
      shadow: opts.shadow ? sh() : undefined,
    });
  }
  function badge(slide, x, y, d, circleColor, iconData) {
    slide.addShape(pres.shapes.OVAL, { x, y, w: d, h: d, fill: { color: circleColor } });
    const pad = d * 0.27;
    slide.addImage({ data: iconData, x: x + pad, y: y + pad, w: d - 2 * pad, h: d - 2 * pad });
  }
  function fit(bx, by, bw, bh, ar) { let w = bw, h = bw / ar; if (h > bh) { h = bh; w = bh * ar; } return { x: bx + (bw - w) / 2, y: by + (bh - h) / 2, w, h }; }
  // framed image: matte panel + image (contain = show all, cover = fill+crop)
  function framed(slide, p, ar, x, y, w, h, opts = {}) {
    const matte = opts.matte || C.matte;
    card(slide, x, y, w, h, matte, { border: opts.border || C.panel2 });
    const t = 0.06;
    if (opts.cover) {
      slide.addImage({ path: p, x: x + t, y: y + t, w: w - 2 * t, h: h - 2 * t, sizing: { type: "cover", w: w - 2 * t, h: h - 2 * t } });
    } else {
      const f = fit(x + t, y + t, w - 2 * t, h - 2 * t, ar);
      slide.addImage({ path: p, x: f.x, y: f.y, w: f.w, h: f.h });
    }
  }
  function placeholder(slide, x, y, w, h, label, kind = "VIDEO") {
    slide.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, rectRadius: 0.07, fill: { color: C.panel }, line: { color: C.cyanD, width: 1.5, dashType: "dash" } });
    const d = 0.9;
    badge(slide, x + w / 2 - d / 2, y + h / 2 - d / 2 - 0.25, d, C.panel2, kind === "VIDEO" ? I.play : I.image);
    slide.addText(`${kind} — add here`, { x, y: y + h / 2 + 0.32, w, h: 0.34, fontFace: F.body, fontSize: 13, bold: true, color: C.cyan, align: "center", charSpacing: 1, margin: 0 });
    if (label) slide.addText(label, { x: x + 0.2, y: y + h / 2 + 0.66, w: w - 0.4, h: 0.5, fontFace: F.body, fontSize: 11.5, italic: true, color: C.muted, align: "center", margin: 0 });
  }
  function dots(slide, cx, cy, sx, sy, count) {
    const cols = [C.cyan, C.coral, C.white, C.cyanD, C.amber, "4E6A86"];
    let seed = 7; const rnd = () => { seed = (seed * 9301 + 49297) % 233280; return seed / 233280; };
    for (let i = 0; i < count; i++) {
      const a = rnd() * Math.PI * 2, r = Math.sqrt(rnd());
      const x = cx + Math.cos(a) * r * sx, y = cy + Math.sin(a) * r * sy;
      const s = 0.035 + rnd() * 0.075;
      slide.addShape(pres.shapes.OVAL, { x, y, w: s, h: s, fill: { color: cols[Math.floor(rnd() * cols.length)], transparency: Math.floor(rnd() * 45) } });
    }
  }
  // line between two points (handles diagonal direction)
  function seg(slide, a, b, o = {}) {
    slide.addShape(pres.shapes.LINE, { x: Math.min(a[0], b[0]), y: Math.min(a[1], b[1]), w: Math.abs(b[0] - a[0]), h: Math.abs(b[1] - a[1]), line: Object.assign({ color: C.panel2, width: 2 }, o), flipH: (b[0] - a[0]) * (b[1] - a[1]) < 0 });
  }
  function poseRing(slide, cx, cy, R, o = {}) {
    const nodes = []; for (let i = 0; i < 10; i++) { const a = -Math.PI / 2 + i * (Math.PI / 5); nodes.push([cx + Math.cos(a) * R, cy + Math.sin(a) * R]); }
    for (let i = 0; i < 10; i++) seg(slide, nodes[i], nodes[(i + 1) % 10], { color: o.ring || C.panel2, width: 2 });
    [[0, 4], [2, 7], [5, 9], [1, 6]].forEach(([i, j]) => seg(slide, nodes[i], nodes[j], { color: C.cyan, width: 1, dashType: "dash" }));
    nodes.forEach((n) => slide.addShape(pres.shapes.OVAL, { x: n[0] - 0.12, y: n[1] - 0.12, w: 0.24, h: 0.24, fill: { color: C.cyan }, line: { color: C.bg, width: 2 } }));
    return nodes;
  }
  let s;

  // =====================================================================
  // 1 — INTRO
  // =====================================================================
  s = pres.addSlide(); bg(s);
  dots(s, W / 2, 3.4, 6.3, 3.3, 150);
  s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 1.45, w: W, h: 4.95, fill: { color: C.bg, transparency: 14 } });
  s.addText("UNIVERSITY SPRINT  ·  PERCEPTION & ROBOTICS", { x: 0, y: 1.55, w: W, h: 0.34, fontFace: F.body, fontSize: 14, bold: true, color: C.cyan, charSpacing: 3, align: "center", margin: 0 });
  s.addText("Street View Robot", { x: 0, y: 2.0, w: W, h: 1.1, fontFace: F.head, fontSize: 60, bold: true, color: C.white, align: "center", margin: 0 });
  s.addText("A mobile robot that drives a room and builds a 360° depth map at every stop.", { x: 0, y: 3.25, w: W, h: 0.5, fontFace: F.body, fontSize: 19, color: C.textL, align: "center", margin: 0 });
  // team chips
  const team = ["Saed Abu Fool", "Saleh Khalil", "Mahmod Stitia", "Ahmad Khalifa", "Ahmad Shalabi"];
  s.addText("TEAM", { x: 0, y: 4.55, w: W, h: 0.3, fontFace: F.body, fontSize: 12, bold: true, color: C.muted, charSpacing: 4, align: "center", margin: 0 });
  const cw = 2.3, cg = 0.16, tot = team.length * cw + (team.length - 1) * cg, x0 = (W - tot) / 2;
  team.forEach((nm, i) => {
    const x = x0 + i * (cw + cg);
    card(s, x, 4.95, cw, 0.66, C.panel, { border: C.panel2 });
    s.addText(nm, { x, y: 4.95, w: cw, h: 0.66, fontFace: F.body, fontSize: 13.5, bold: true, color: C.textL, align: "center", valign: "middle", margin: 0 });
  });
  s.addText([{ text: "Instructor:  ", options: { color: C.muted } }, { text: "Mr. Rajaei Khatib", options: { bold: true, color: C.cyan } }], { x: 0, y: 5.95, w: W, h: 0.4, fontFace: F.body, fontSize: 15, align: "center", margin: 0 });

  // =====================================================================
  // 2 — GOAL
  // =====================================================================
  s = pres.addSlide(); bg(s);
  head(s, "THE GOAL", "Scan the surroundings → a 360° depth map");
  s.addText("Park anywhere, spin once, and turn what the camera sees into a single 3D depth map of the room.", { x: 0.95, y: 1.7, w: 6.0, h: 1.1, fontFace: F.body, fontSize: 17, color: C.textL, lineSpacingMultiple: 1.15, margin: 0 });
  const goal = [
    [I.gamepad, C.cyan, "Drive to a spot", "manual or autonomous"],
    [I.sync, C.coral, "Spin 360° in place", "10 stereo captures"],
    [I.cubes, C.green, "Fuse into one cloud", "a navigable 3D depth map"],
  ];
  goal.forEach((g, i) => {
    const y = 3.0 + i * 1.12;
    badge(s, 0.95, y, 0.74, g[1], g[0]);
    s.addText([{ text: g[2], options: { bold: true, color: C.white, breakLine: true } }, { text: g[3], options: { color: C.muted, fontSize: 13 } }], { x: 1.9, y: y - 0.05, w: 4.9, h: 0.8, fontFace: F.body, fontSize: 16, valign: "middle", margin: 0 });
    if (i < goal.length - 1) seg(s, [1.32, y + 0.74], [1.32, y + 1.12], { color: C.panel2, width: 2 });
  });
  // right: room + scan-stops diagram
  card(s, 7.35, 1.7, 5.25, 4.95, C.panel, { border: C.panel2 });
  s.addText("ONE ROOM, MANY STOPS", { x: 7.35, y: 1.92, w: 5.25, h: 0.3, fontFace: F.body, fontSize: 11, bold: true, color: C.muted, charSpacing: 2, align: "center", margin: 0 });
  s.addShape(pres.shapes.RECTANGLE, { x: 7.75, y: 2.45, w: 4.45, h: 3.9, fill: { color: C.matte }, line: { color: C.panel2, width: 1 } });
  const stops = [[8.7, 3.35], [11.1, 3.2], [9.8, 4.4], [8.5, 5.5], [11.3, 5.45]];
  for (let i = 0; i < stops.length - 1; i++) seg(s, stops[i], stops[i + 1], { color: "3A557A", width: 1, dashType: "dash" });
  stops.forEach((p, i) => {
    const col = i === 2 ? C.coral : C.cyan;
    s.addShape(pres.shapes.OVAL, { x: p[0] - 0.4, y: p[1] - 0.4, w: 0.8, h: 0.8, fill: { color: col, transparency: 84 }, line: { color: col, width: 1, dashType: "dash" } });
    s.addShape(pres.shapes.OVAL, { x: p[0] - 0.1, y: p[1] - 0.1, w: 0.2, h: 0.2, fill: { color: col } });
  });
  s.addText("● stop      ◌ 360° scan", { x: 7.75, y: 6.0, w: 4.45, h: 0.3, fontFace: F.body, fontSize: 10.5, color: C.muted, align: "center", margin: 0 });
  footer(s, 2);

  // =====================================================================
  // 3 — HARDWARE
  // =====================================================================
  s = pres.addSlide(); bg(s);
  head(s, "HARDWARE", "What the robot is made of");
  const hw = [
    { ic: I.robot, col: C.cyanD, t: "Raspbot V2", d: ["4× Mecanum wheels", "omnidirectional + spin", "I2C motor board"] },
    { ic: I.chip, col: C.cyan, t: "Raspberry Pi 5", d: ["8 GB RAM", "runs the whole stack", "headless, over Wi-Fi"] },
    { ic: I.camera, col: C.coral, t: "RealSense D405", d: ["stereo IR, 18 mm base", "factory-calibrated", "passive — no projector"] },
  ];
  hw.forEach((c, i) => {
    const x = 0.7 + i * 4.05;
    card(s, x, 1.85, 3.75, 3.35, C.panel, { border: C.panel2, shadow: true });
    badge(s, x + 0.35, 2.2, 0.92, c.col, c.ic);
    s.addText(c.t, { x: x + 0.35, y: 3.28, w: 3.1, h: 0.45, fontFace: F.head, fontSize: 20, bold: true, color: C.white, margin: 0 });
    s.addText(c.d.map((l, j) => ({ text: l, options: { bullet: { code: "2022" }, color: C.muted, breakLine: j < c.d.length - 1, paraSpaceAfter: 6 } })), { x: x + 0.35, y: 3.82, w: 3.15, h: 1.2, fontFace: F.body, fontSize: 13.5, margin: 0 });
  });
  card(s, 0.7, 5.5, 11.85, 1.1, C.panel, { border: C.panel2 });
  s.addText("ALSO ONBOARD", { x: 1.0, y: 5.75, w: 2.4, h: 0.55, fontFace: F.body, fontSize: 11.5, bold: true, color: C.cyan, charSpacing: 2, valign: "middle", margin: 0 });
  badge(s, 4.0, 5.74, 0.62, C.panel2, I.wave);
  s.addText([{ text: "Ultrasonic rangefinder", options: { bold: true, color: C.textL, breakLine: true } }, { text: "obstacle distance", options: { color: C.muted, fontSize: 11 } }], { x: 4.75, y: 5.7, w: 3.5, h: 0.75, fontFace: F.body, fontSize: 13, valign: "middle", margin: 0 });
  badge(s, 8.3, 5.74, 0.62, C.panel2, I.road);
  s.addText([{ text: "4× IR line-tracking sensors", options: { bold: true, color: C.textL, breakLine: true } }, { text: "ground reference / line following", options: { color: C.muted, fontSize: 11 } }], { x: 9.05, y: 5.7, w: 3.5, h: 0.75, fontFace: F.body, fontSize: 13, valign: "middle", margin: 0 });
  footer(s, 3);

  // =====================================================================
  // 4 — TWO DRIVE MODES  (IMPLEMENTATION)
  // =====================================================================
  s = pres.addSlide(); bg(s);
  head(s, "IMPLEMENTATION", "Two ways to drive");
  const modes = [
    { ic: I.gamepad, col: C.cyan, tag: "MODE 1", t: "Manual control", lines: ["Real-time WASD — omnidirectional + spin.", "One keypress fires the scan routine.", "Also drivable from the web dashboard."] },
    { ic: I.route, col: C.coral, tag: "MODE 2", t: "Autonomous line-following", lines: ["Follows dark tape via 4 IR sensors.", "Cross “stop markers” mark a scan point.", "Halt → scan → resume to the next."] },
  ];
  modes.forEach((m, i) => {
    const x = 0.7 + i * 6.1;
    card(s, x, 1.85, 5.75, 3.95, C.panel, { border: C.panel2, shadow: true });
    s.addShape(pres.shapes.RECTANGLE, { x, y: 1.85, w: 5.75, h: 0.12, fill: { color: m.col } });
    badge(s, x + 0.4, 2.28, 0.85, m.col, m.ic);
    s.addText(m.tag, { x: x + 1.45, y: 2.35, w: 4, h: 0.3, fontFace: F.body, fontSize: 12, bold: true, color: m.col, charSpacing: 3, margin: 0 });
    s.addText(m.t, { x: x + 1.45, y: 2.63, w: 4.1, h: 0.55, fontFace: F.head, fontSize: 20, bold: true, color: C.white, margin: 0 });
    s.addText(m.lines.map((l) => ({ text: l, options: { bullet: { code: "2022", indent: 14 }, color: C.textL, breakLine: true, paraSpaceAfter: 11 } })), { x: x + 0.45, y: 3.5, w: 5.0, h: 2.1, fontFace: F.body, fontSize: 14.5, margin: 0 });
  });
  card(s, 0.7, 6.05, 11.85, 0.78, C.panel2);
  badge(s, 0.95, 6.16, 0.56, C.cyan, I.sync);
  s.addText([{ text: "Both modes feed the same routine:  ", options: { color: C.textL } }, { text: "rotate → capture → merge", options: { bold: true, color: C.cyan } }], { x: 1.65, y: 6.05, w: 10.6, h: 0.78, fontFace: F.body, fontSize: 16, valign: "middle", margin: 0 });
  footer(s, 4);

  // =====================================================================
  // 5 — SENSOR PLACEMENT  (IMPLEMENTATION) — 2 photo placeholders
  // =====================================================================
  s = pres.addSlide(); bg(s);
  head(s, "IMPLEMENTATION", "A custom 3D-printed sensor mount");
  s.addText([
    { text: "We modelled a mount in CAD and 3D-printed it", options: { bold: true, color: C.white } },
    { text: " to fix the D405 + pan/tilt camera on the car at the right height and angle.", options: { color: C.textL } },
  ], { x: 0.95, y: 1.7, w: 11.5, h: 0.5, fontFace: F.body, fontSize: 16.5, margin: 0 });
  const sp = [[I.cad, "Designed in CAD", "modelled to fit the chassis + camera"], [I.print, "3D-printed", "mounted on the robot"]];
  sp.forEach((p, i) => {
    const x = 0.95 + i * 5.95;
    placeholder(s, x, 2.55, 5.5, 3.5, p[2], "PHOTO");
    badge(s, x + 0.25, 2.78, 0.6, i === 0 ? C.cyan : C.coral, p[0]);
    s.addText(p[1], { x: x + 0.95, y: 2.78, w: 4.3, h: 0.6, fontFace: F.head, fontSize: 17, bold: true, color: C.white, valign: "middle", margin: 0 });
  });
  footer(s, 5);

  // =====================================================================
  // 6 — CAR AUTO-ROTATE  (IMPLEMENTATION)
  // =====================================================================
  s = pres.addSlide(); bg(s);
  head(s, "IMPLEMENTATION", "Spin 360° · capture 10 shots");
  s.addText("No IMU or wheel encoders — the spin is open-loop: timed motor pulses, calibrated once per floor. Stop every ~36°, take a stereo shot, repeat ×10.", { x: 0.95, y: 1.65, w: 5.2, h: 1.5, fontFace: F.body, fontSize: 16, color: C.textL, lineSpacingMultiple: 1.18, margin: 0 });
  // rotation dial
  const dcx = 3.55, dcy = 4.7, dR = 1.5;
  s.addShape(pres.shapes.OVAL, { x: dcx - dR, y: dcy - dR, w: 2 * dR, h: 2 * dR, fill: { color: C.panel }, line: { color: C.panel2, width: 2 } });
  for (let i = 0; i < 10; i++) {
    const a = -Math.PI / 2 + i * (Math.PI / 5);
    const px = dcx + Math.cos(a) * dR, py = dcy + Math.sin(a) * dR;
    s.addShape(pres.shapes.OVAL, { x: px - 0.13, y: py - 0.13, w: 0.26, h: 0.26, fill: { color: i === 0 ? C.coral : C.cyan }, line: { color: C.bg, width: 2 } });
  }
  badge(s, dcx - 0.33, dcy - 0.33, 0.66, C.panel2, I.robot);
  s.addText("10 stops · 36°", { x: dcx - 1.5, y: dcy + dR + 0.12, w: 3.0, h: 0.3, fontFace: F.body, fontSize: 12, bold: true, color: C.muted, align: "center", margin: 0 });
  // filmstrip of real captured shots
  s.addText("WHAT IT CAPTURES (real shots)", { x: 6.5, y: 2.05, w: 6, h: 0.3, fontFace: F.body, fontSize: 11, bold: true, color: C.cyan, charSpacing: 2, margin: 0 });
  const fsW = 1.92, fsGap = 0.16, fsH = fsW / AR.scan, fy = 2.5;
  [0, 3, 6].forEach((n, i) => {
    const x = 6.5 + i * (fsW + fsGap);
    framed(s, IMG.scan(n), AR.scan, x, fy, fsW, fsH, { cover: true });
    s.addText(`shot ${n}`, { x, y: fy + fsH + 0.04, w: fsW, h: 0.28, fontFace: F.body, fontSize: 11, color: C.muted, align: "center", margin: 0 });
  });
  card(s, 6.55, 5.35, 6.0, 1.25, C.panel2);
  s.addText([
    { text: "Each shot → ", options: { color: C.textL } },
    { text: "captures/scan_<ts>/shot_NN/", options: { fontFace: F.mono, color: C.cyan, fontSize: 13 } },
    { text: "  with depth, the IR pair, intrinsics, and its cumulative angle.", options: { color: C.textL } },
  ], { x: 6.85, y: 5.35, w: 5.5, h: 1.25, fontFace: F.body, fontSize: 13.5, valign: "middle", lineSpacingMultiple: 1.1, margin: 0 });
  footer(s, 6);

  // =====================================================================
  // 7 — DEPTH ALGORITHM (SGBM)  (IMPLEMENTATION) — real images
  // =====================================================================
  s = pres.addSlide(); bg(s);
  head(s, "IMPLEMENTATION", "Our own stereo depth — SGBM");
  s.addText("Two IR views of the same scene are offset. Block matching finds each pixel’s left↔right shift (disparity); closer points shift more.", { x: 0.95, y: 1.62, w: 11.6, h: 0.6, fontFace: F.body, fontSize: 15.5, color: C.textL, margin: 0 });
  const fw = 3.45, fh = 2.05, ry = 2.55;
  const labels = ["IR — left", "IR — right", "Depth — our SGBM"];
  const srcs = [[IMG.irL, AR.irL], [IMG.irR, AR.irR], [IMG.depth, AR.depth]];
  const xs = [0.95, 5.0, 9.05];
  xs.forEach((x, i) => {
    framed(s, srcs[i][0], srcs[i][1], x, ry, fw, fh);
    s.addText(labels[i], { x, y: ry + fh + 0.05, w: fw, h: 0.3, fontFace: F.body, fontSize: 12, bold: true, color: i === 2 ? C.cyan : C.muted, align: "center", margin: 0 });
  });
  s.addImage({ data: I.arrow, x: 4.5, y: ry + fh / 2 - 0.22, w: 0.42, h: 0.42 });
  s.addImage({ data: I.arrow, x: 8.55, y: ry + fh / 2 - 0.22, w: 0.42, h: 0.42 });
  // equation + notes band
  card(s, 0.95, 5.35, 11.6, 1.25, C.panel2);
  s.addText([{ text: "depth", options: { color: C.white } }, { text: " = ", options: { color: C.muted } }, { text: "f · B", options: { color: C.cyan, bold: true } }, { text: " / ", options: { color: C.muted } }, { text: "disparity", options: { color: C.coral, bold: true } }], { x: 1.25, y: 5.35, w: 4.3, h: 1.25, fontFace: F.mono, fontSize: 23, bold: true, valign: "middle", margin: 0 });
  s.addText([
    { text: "Semi-Global Block Matching", options: { bold: true, color: C.textL, breakLine: true } },
    { text: "smooths matches along many directions; optional WLS filter fills holes.", options: { color: C.muted } },
  ], { x: 6.0, y: 5.35, w: 6.3, h: 1.25, fontFace: F.body, fontSize: 13.5, valign: "middle", lineSpacingMultiple: 1.08, margin: 0 });
  footer(s, 7);

  // =====================================================================
  // 8 — 360 PART 1: FEATURE MATCHING  (IMPLEMENTATION)
  // =====================================================================
  s = pres.addSlide(); bg(s);
  head(s, "IMPLEMENTATION", "10 shots → 360°  ·  ① match features");
  s.addText("The robot’s turn is unreliable, so we don’t trust the angle — we measure it from what the cameras saw. First, find and match features between overlapping shots.", { x: 0.95, y: 1.62, w: 11.6, h: 0.65, fontFace: F.body, fontSize: 15.5, color: C.textL, lineSpacingMultiple: 1.1, margin: 0 });
  const fm = [
    [I.bolt, C.amber, "CLAHE-enhance the IR", "the D405 IR is dim (~55/255) — boost contrast first"],
    [I.search, C.cyan, "Detect ORB / SIFT keypoints", "thousands of corners per shot"],
    [I.graph, C.coral, "Match overlapping pairs", "filter false matches with a 2D fundamental test"],
  ];
  fm.forEach((b, i) => {
    const y = 2.5 + i * 1.05;
    badge(s, 0.95, y, 0.66, b[1], b[0]);
    s.addText([{ text: b[2], options: { bold: true, color: C.white, breakLine: true } }, { text: b[3], options: { color: C.muted, fontSize: 12.5 } }], { x: 1.8, y: y - 0.05, w: 5.6, h: 0.95, fontFace: F.body, fontSize: 15, valign: "middle", margin: 0 });
  });
  // CLAHE win callout (right)
  card(s, 7.7, 2.45, 4.9, 3.35, C.panel, { border: C.panel2 });
  s.addText("WHY CLAHE MATTERS", { x: 7.95, y: 2.68, w: 4.4, h: 0.3, fontFace: F.body, fontSize: 11, bold: true, color: C.cyan, charSpacing: 2, margin: 0 });
  s.addText("Raw, dim shots give the detector almost nothing — the ring breaks. Contrast-boosting first reveals the real texture:", { x: 7.95, y: 3.0, w: 4.4, h: 1.0, fontFace: F.body, fontSize: 13, color: C.textL, lineSpacingMultiple: 1.08, margin: 0 });
  const winv = [["53", "992", "keypoints / shot"], ["4/10", "10/10", "shots registered"]];
  winv.forEach((wv, i) => {
    const y = 4.1 + i * 0.85;
    s.addText(wv[0], { x: 7.95, y, w: 1.5, h: 0.7, fontFace: F.head, fontSize: 26, bold: true, color: C.muted, align: "center", valign: "middle", margin: 0 });
    s.addImage({ data: I.arrow, x: 9.5, y: y + 0.18, w: 0.45, h: 0.45 });
    s.addText(wv[1], { x: 10.0, y, w: 1.5, h: 0.7, fontFace: F.head, fontSize: 26, bold: true, color: C.cyan, align: "center", valign: "middle", margin: 0 });
    s.addText(wv[2], { x: 11.45, y, w: 1.0, h: 0.7, fontFace: F.body, fontSize: 10.5, color: C.muted, valign: "middle", margin: 0 });
  });
  footer(s, 8);

  // =====================================================================
  // 9 — 360 PART 2: SOLVE GEOMETRY + RESULT  (IMPLEMENTATION)
  // =====================================================================
  s = pres.addSlide(); bg(s);
  head(s, "IMPLEMENTATION", "10 shots → 360°  ·  ② solve the geometry");
  const g2 = [
    "Lift each matched feature to 3D through the depth map.",
    "Per pair: Kabsch + RANSAC → a rigid transform (no angle guess).",
    "Pose graph: neighbours = odometry, the rest = loop closures.",
    "Global optimisation spreads drift → fuse, voxel 5 mm.",
  ];
  g2.forEach((t, i) => {
    const y = 1.75 + i * 0.82;
    s.addShape(pres.shapes.OVAL, { x: 0.95, y, w: 0.48, h: 0.48, fill: { color: i === g2.length - 1 ? C.coral : C.cyan } });
    s.addText(String(i + 1), { x: 0.95, y, w: 0.48, h: 0.48, fontFace: F.head, fontSize: 16, bold: true, color: C.bg, align: "center", valign: "middle", margin: 0 });
    s.addText(t, { x: 1.6, y: y - 0.12, w: 4.5, h: 0.75, fontFace: F.body, fontSize: 14, color: C.textL, valign: "middle", lineSpacingMultiple: 1.0, margin: 0 });
  });
  // pose graph small
  s.addText("pose graph", { x: 1.5, y: 4.95, w: 2.8, h: 0.3, fontFace: F.body, fontSize: 10.5, color: C.muted, align: "center", margin: 0 });
  poseRing(s, 2.9, 6.02, 0.72);
  // result image (the ring) on the right
  const rb = fit(6.4, 1.7, 6.2, 4.0, AR.ring);
  card(s, rb.x - 0.12, rb.y - 0.12, rb.w + 0.24, rb.h + 0.24, C.white, { shadow: true });
  s.addImage({ path: IMG.ring, x: rb.x, y: rb.y, w: rb.w, h: rb.h });
  card(s, 6.4, 5.95, 6.2, 0.72, C.panel2);
  s.addText([{ text: "Result:  ", options: { color: C.muted } }, { text: "all 10 shots in one connected 360° cloud", options: { bold: true, color: C.cyan } }, { text: "   (red = near, blue = far)", options: { color: C.muted, fontSize: 12 } }], { x: 6.6, y: 5.95, w: 5.9, h: 0.72, fontFace: F.body, fontSize: 14, valign: "middle", margin: 0 });
  footer(s, 9);

  // =====================================================================
  // 10 — VIDEO RESULT  (IMPLEMENTATION)
  // =====================================================================
  s = pres.addSlide(); bg(s);
  head(s, "IMPLEMENTATION", "Result — a full room scan");
  placeholder(s, 2.7, 1.75, 7.93, 4.6, "Robot drives, spins, and the 360° cloud builds up", "VIDEO");
  footer(s, 10);

  // =====================================================================
  // 11 — OBJECT SCAN  (IMPLEMENTATION) — real orbit shots
  // =====================================================================
  s = pres.addSlide(); bg(s);
  head(s, "IMPLEMENTATION", "Bonus — scan a single 3D object", C.violet);
  s.addText("Instead of mapping a room, orbit one object (D405 sweet spot: 7–50 cm) and reconstruct it as a clean point cloud + a textured mesh.", { x: 0.95, y: 1.62, w: 11.6, h: 0.6, fontFace: F.body, fontSize: 15.5, color: C.textL, margin: 0 });
  // real orbit filmstrip
  const obW = 2.7, obH = obW / AR.orbit, oy = 2.5;
  [0, 4, 6, 9].forEach((n, i) => {
    const x = 0.95 + i * (obW + 0.18);
    framed(s, IMG.orbit(n), AR.orbit, x, oy, obW, obH, { cover: true });
  });
  s.addText("Robot orbits the object — background segmented out, object kept (real captures)", { x: 0.95, y: oy + obH + 0.08, w: 11.6, h: 0.3, fontFace: F.body, fontSize: 11.5, italic: true, color: C.muted, margin: 0 });
  // pipeline row
  const op = [[I.segment, "Segment"], [I.sync, "Angle prior"], [I.graph, "Multiway ICP + loop"], [I.mesh, "Poisson mesh"]];
  const opY = oy + obH + 0.6, opW = 2.78;
  op.forEach((p, i) => {
    const x = 0.95 + i * (opW + 0.16);
    card(s, x, opY, opW, 1.05, C.panel, { border: C.panel2 });
    badge(s, x + 0.22, opY + 0.24, 0.56, i === op.length - 1 ? C.violet : C.cyanD, p[0]);
    s.addText(`${i + 1}. ${p[1]}`, { x: x + 0.9, y: opY, w: opW - 1.0, h: 1.05, fontFace: F.body, fontSize: 13.5, bold: true, color: C.textL, valign: "middle", margin: 0 });
    if (i < op.length - 1) s.addImage({ data: I.arrow, x: x + opW - 0.02, y: opY + 0.32, w: 0.18, h: 0.4 });
  });
  footer(s, 11);

  // =====================================================================
  // 12 — OBJECT SCAN DEMO  (IMPLEMENTATION) — video + photo placeholders
  // =====================================================================
  s = pres.addSlide(); bg(s);
  head(s, "IMPLEMENTATION", "Object scan — demo & result", C.violet);
  placeholder(s, 0.95, 1.9, 6.0, 4.4, "Scanning the object (orbit / turntable)", "VIDEO");
  placeholder(s, 7.35, 1.9, 5.0, 4.4, "The reconstructed 3D mesh", "PHOTO");
  footer(s, 12);

  // =====================================================================
  // 13 — LINE FOLLOWING DEMO  (IMPLEMENTATION)
  // =====================================================================
  s = pres.addSlide(); bg(s);
  head(s, "IMPLEMENTATION", "Autonomous line-following");
  // left: explanation + sensor/tape diagram
  s.addText("A PID state machine reads the 4 IR sensors and steers to keep the tape centred. A cross marker (all 4 trip) = a scan stop.", { x: 0.95, y: 1.7, w: 5.6, h: 1.1, fontFace: F.body, fontSize: 15.5, color: C.textL, lineSpacingMultiple: 1.15, margin: 0 });
  // mini tape diagram
  card(s, 0.95, 3.05, 5.6, 3.25, C.panel, { border: C.panel2 });
  s.addShape(pres.shapes.RECTANGLE, { x: 1.3, y: 3.4, w: 4.9, h: 2.55, fill: { color: C.matte } });
  // curved-ish tape (segments)
  const tape = [[1.6, 5.7], [2.5, 5.0], [3.5, 4.6], [4.6, 4.9], [5.6, 4.3]];
  for (let i = 0; i < tape.length - 1; i++) seg(s, tape[i], tape[i + 1], { color: C.textL, width: 5 });
  // stop marker (cross)
  s.addShape(pres.shapes.RECTANGLE, { x: 4.45, y: 4.55, w: 0.6, h: 0.14, fill: { color: C.coral } });
  s.addShape(pres.shapes.RECTANGLE, { x: 4.68, y: 4.32, w: 0.14, h: 0.6, fill: { color: C.coral } });
  // sensors row
  [0, 1, 2, 3].forEach((i) => s.addShape(pres.shapes.OVAL, { x: 2.0 + i * 0.32, y: 5.45, w: 0.16, h: 0.16, fill: { color: C.cyan } }));
  s.addText("— tape   ＋ stop marker   ● 4 IR sensors", { x: 1.3, y: 5.98, w: 4.9, h: 0.28, fontFace: F.body, fontSize: 10.5, color: C.muted, margin: 0 });
  // right: demo video placeholder
  placeholder(s, 6.85, 1.7, 5.7, 4.6, "Robot following the tape to a stop marker", "VIDEO");
  footer(s, 13);

  // =====================================================================
  // 14 — SUMMARY OF FEATURES
  // =====================================================================
  s = pres.addSlide(); bg(s);
  head(s, "RECAP", "Everything the robot does");
  const feats = [
    [I.map, C.cyan, "360° room depth map", "spin + fuse into one 3D cloud"],
    [I.ruler, C.coral, "Custom SGBM depth", "our own stereo depth, from scratch"],
    [I.gamepad, C.green, "Manual driving", "WASD teleop + web dashboard"],
    [I.route, C.amber, "Autonomous line-follow", "PID + stop-marker scanning"],
    [I.cube, C.violet, "3D object scan", "orbit → point cloud + mesh"],
    [I.print, C.cyanD, "3D-printed mount", "custom CAD sensor rig"],
  ];
  const gw = 3.85, gh = 1.95, gx0 = 0.7, gy0 = 1.85, gxg = 0.2, gyg = 0.22;
  feats.forEach((f, i) => {
    const c = i % 3, r = Math.floor(i / 3);
    const x = gx0 + c * (gw + gxg), y = gy0 + r * (gh + gyg);
    card(s, x, y, gw, gh, C.panel, { border: C.panel2, shadow: true });
    s.addShape(pres.shapes.RECTANGLE, { x, y, w: 0.1, h: gh, fill: { color: f[1] } });
    badge(s, x + 0.35, y + 0.35, 0.7, f[1], f[0]);
    s.addText(f[2], { x: x + 0.35, y: y + 1.12, w: gw - 0.6, h: 0.4, fontFace: F.head, fontSize: 16.5, bold: true, color: C.white, margin: 0 });
    s.addText(f[3], { x: x + 0.35, y: y + 1.5, w: gw - 0.55, h: 0.35, fontFace: F.body, fontSize: 12.5, color: C.muted, margin: 0 });
  });
  footer(s, 14);

  // =====================================================================
  // 15 — THANK YOU
  // =====================================================================
  s = pres.addSlide(); bg(s);
  dots(s, W / 2, 3.75, 5.9, 3.2, 150);
  s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 2.5, w: W, h: 2.6, fill: { color: C.bg, transparency: 16 } });
  s.addText("Thank you", { x: 0, y: 2.65, w: W, h: 1.0, fontFace: F.head, fontSize: 56, bold: true, color: C.white, align: "center", margin: 0 });
  s.addText("An autonomous robot that turns a room — or an object — into 3D.", { x: 0, y: 3.9, w: W, h: 0.5, fontFace: F.body, fontSize: 19, color: C.cyan, align: "center", margin: 0 });
  s.addText("Questions?", { x: 0, y: 4.7, w: W, h: 0.4, fontFace: F.body, fontSize: 16, italic: true, color: C.muted, align: "center", margin: 0 });

  await pres.writeFile({ fileName: OUT });
  console.log("WROTE", OUT);
}
main().catch((e) => { console.error(e); process.exit(1); });
