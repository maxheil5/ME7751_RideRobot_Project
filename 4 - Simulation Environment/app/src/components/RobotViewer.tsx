import { Html, Line, OrbitControls } from "@react-three/drei";
import { Canvas } from "@react-three/fiber";
import { useEffect, useMemo, useRef, useState } from "react";
import { Group, Mesh, MeshPhongMaterial } from "three";
import type { LoadingManager, Object3D } from "three";
import URDFLoader from "urdf-loader";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";

interface RobotViewerProps {
  linkFrames: number[][][];
  currentQ: number[];
  currentPoseText: string;
}

interface UrdfRobotLike extends Object3D {
  joints?: Record<string, unknown>;
  setJointValue?: (jointName: string, value: number) => void;
}

type MeshDone = (mesh: Object3D, err?: Error) => void;

// Visual mapping from KR500 planner joints to surrogate URDF joints.
// This keeps the URDF in a stable posture at planner home q=[0,-90,90,0,0,0].
const URDF_HOME_Q_DEG = [0, -90, 90, 0, 0, 0];
const URDF_JOINT_SCALE = [1, 1, 1, 1, 1, 1];
const URDF_JOINT_BIAS_DEG = [0, 0, 0, 0, 0, 0];

const JOINT_NAME_CANDIDATES: string[][] = [
  ["joint_1", "joint1", "joint_a1", "a1", "A1"],
  ["joint_2", "joint2", "joint_a2", "a2", "A2"],
  ["joint_3", "joint3", "joint_a3", "a3", "A3"],
  ["joint_4", "joint4", "joint_a4", "a4", "A4"],
  ["joint_5", "joint5", "joint_a5", "a5", "A5"],
  ["joint_6", "joint6", "joint_a6", "a6", "A6"],
  ["joint_7", "joint7", "joint_a7", "a7", "A7"],
];

function findJointName(jointKeys: string[], index: number): string | null {
  const jointNumber = String(index + 1);
  const candidates = JOINT_NAME_CANDIDATES[index] || [];
  for (const candidate of candidates) {
    const hit = jointKeys.find((k) => k.toLowerCase() === candidate.toLowerCase());
    if (hit) {
      return hit;
    }
  }

  const fuzzy = jointKeys.find((k) => {
    const key = k.toLowerCase();
    return key.endsWith(`joint_${jointNumber}`) || key.endsWith(`joint${jointNumber}`) || key.includes(`joint_${jointNumber}`);
  });
  if (fuzzy) {
    return fuzzy;
  }

  return null;
}

function mapPlannerJointsToUrdf(qDeg: number[]): number[] {
  return qDeg.map((q, i) => (q - URDF_HOME_Q_DEG[i]) * URDF_JOINT_SCALE[i] + URDF_JOINT_BIAS_DEG[i]);
}

function applyUrdfJointValues(robot: UrdfRobotLike, qDeg: number[]) {
  if (!robot?.setJointValue) {
    return;
  }

  const mappedQDeg = mapPlannerJointsToUrdf(qDeg);
  const jointKeys = Object.keys(robot.joints || {});
  for (let i = 0; i < 6; i += 1) {
    const jointName = findJointName(jointKeys, i);
    if (!jointName) {
      continue;
    }
    robot.setJointValue(jointName, (mappedQDeg[i] * Math.PI) / 180);
  }

  // Keep extra joints (such as a 7th axis) deterministic for non-6DOF URDF models.
  const joint7 = findJointName(jointKeys, 6);
  if (joint7) {
    robot.setJointValue(joint7, 0);
  }

  robot.updateMatrixWorld(true);
}

function loadUrdfMesh(path: string, manager: LoadingManager, done: MeshDone) {
  const lower = path.toLowerCase();

  if (lower.endsWith(".stl")) {
    const loader = new STLLoader(manager);
    loader.load(
      path,
      (geom) => {
        const mesh = new Mesh(geom, new MeshPhongMaterial({ color: 0xa0aab8 }));
        done(mesh);
      },
      undefined,
      (err) => {
        const error = err instanceof Error ? err : new Error(String(err));
        done(new Group(), error);
      },
    );
    return;
  }

  if (lower.endsWith(".obj")) {
    const loader = new OBJLoader(manager);
    loader.load(
      path,
      (obj) => done(obj),
      undefined,
      (err) => {
        const error = err instanceof Error ? err : new Error(String(err));
        done(new Group(), error);
      },
    );
    return;
  }

  done(new Group(), new Error(`Unsupported mesh extension for ${path}`));
}

function robotPointToScene(pointMm: [number, number, number]): [number, number, number] {
  // Robot FK uses Z-up. Three.js scene uses Y-up.
  // Map [x, y, z]_robot(mm) -> [x, y, z]_scene(m).
  return [pointMm[0] / 1000, pointMm[2] / 1000, -pointMm[1] / 1000];
}

function UrdfModel({
  url,
  packageRoot,
  currentQ,
  onReadyState,
}: {
  url: string;
  packageRoot: string;
  currentQ: number[];
  onReadyState: (state: "loaded" | "failed") => void;
}) {
  const [robot, setRobot] = useState<UrdfRobotLike | null>(null);

  useEffect(() => {
    const loader = new URDFLoader();
    // Supports ROS-style package:// URIs in URDF mesh paths.
    (loader as unknown as { packages?: string }).packages = packageRoot;
    (loader as unknown as { loadMeshCb?: (path: string, manager: LoadingManager, done: MeshDone) => void }).loadMeshCb = loadUrdfMesh;
    loader.load(
      url,
      (loaded) => {
        const urdfRobot = loaded as UrdfRobotLike;
        setRobot(urdfRobot);
        // Mesh assets may continue attaching asynchronously after parse.
        onReadyState("loaded");
      },
      undefined,
      () => {
        setRobot(null);
        onReadyState("failed");
      },
    );
  }, [url, packageRoot, onReadyState]);

  useEffect(() => {
    if (!robot) {
      return;
    }
    applyUrdfJointValues(robot, currentQ);
  }, [robot, currentQ]);

  if (!robot) {
    return null;
  }

  return <primitive object={robot} scale={1} rotation={[-Math.PI / 2, 0, 0]} />;
}

function ProceduralRobot({ linkFrames }: { linkFrames: number[][][] }) {
  const points = useMemo(() => {
    const pts: [number, number, number][] = [[0, 0, 0]];
    for (const frame of linkFrames) {
      pts.push(robotPointToScene([frame[0][3], frame[1][3], frame[2][3]]));
    }
    return pts;
  }, [linkFrames]);

  const tcp = points[points.length - 1] || [0, 0, 0];

  return (
    <group>
      {points.map((point, idx) => (
        <mesh key={`joint-${idx}`} position={point}>
          <sphereGeometry args={[0.04, 20, 20]} />
          <meshStandardMaterial color={idx === points.length - 1 ? "#f97316" : "#1d4ed8"} roughness={0.4} metalness={0.2} />
        </mesh>
      ))}

      {points.slice(1).map((point, idx) => (
        <Line key={`link-${idx}`} points={[points[idx], point]} color="#000000" lineWidth={3.6} />
      ))}

      <Line points={[tcp, [tcp[0] + 0.2, tcp[1], tcp[2]]]} color="#ef4444" lineWidth={2} />
      <Line points={[tcp, [tcp[0], tcp[1] + 0.2, tcp[2]]]} color="#22c55e" lineWidth={2} />
      <Line points={[tcp, [tcp[0], tcp[1], tcp[2] + 0.2]]} color="#3b82f6" lineWidth={2} />
    </group>
  );
}

export default function RobotViewer({ linkFrames, currentQ, currentPoseText }: RobotViewerProps) {
  // URDF stays muted unless explicitly enabled in app/.env.
  const urdfUrl = import.meta.env.VITE_URDF_URL as string | undefined;
  const urdfPackageRoot = (import.meta.env.VITE_URDF_PACKAGES as string | undefined) || "/assets/robot_urdf/kuka_iiwa";
  const [urdfState, setUrdfState] = useState<"idle" | "loaded" | "failed">("idle");
  const cameraRef = useRef<any>(null);
  const controlsRef = useRef<any>(null);

  function recenterView() {
    if (!cameraRef.current || !controlsRef.current) {
      return;
    }

    const points = linkFrames.map((frame) => robotPointToScene([frame[0][3], frame[1][3], frame[2][3]]));
    points.push([0, 0, 0]);

    let minX = Number.POSITIVE_INFINITY;
    let minY = Number.POSITIVE_INFINITY;
    let minZ = Number.POSITIVE_INFINITY;
    let maxX = Number.NEGATIVE_INFINITY;
    let maxY = Number.NEGATIVE_INFINITY;
    let maxZ = Number.NEGATIVE_INFINITY;

    for (const p of points) {
      minX = Math.min(minX, p[0]);
      minY = Math.min(minY, p[1]);
      minZ = Math.min(minZ, p[2]);
      maxX = Math.max(maxX, p[0]);
      maxY = Math.max(maxY, p[1]);
      maxZ = Math.max(maxZ, p[2]);
    }

    const centerX = (minX + maxX) * 0.5;
    const centerY = (minY + maxY) * 0.5;
    const centerZ = (minZ + maxZ) * 0.5;

    const span = Math.max(maxX - minX, maxY - minY, maxZ - minZ, 0.8);
    const distance = Math.max(2.0, span * 2.6);

    controlsRef.current.target.set(centerX, centerY, centerZ);
    cameraRef.current.position.set(centerX + distance, centerY + distance * 0.75, centerZ + distance);
    cameraRef.current.updateProjectionMatrix();
    controlsRef.current.update();
  }

  return (
    <section className="panel viewer-panel">
      <div className="panel-header">
        <h3>Robot Viewer</h3>
        <span className="muted">Three.js (hybrid: URDF if available, procedural chain always)</span>
      </div>

      <div className="viewer-meta">
        <p>
          q = [{currentQ.map((v) => v.toFixed(1)).join(", ")}]
        </p>
        <p>{currentPoseText}</p>
        {urdfUrl ? <p className="muted">URDF state: {urdfState}</p> : <p className="muted">URDF state: not configured</p>}
      </div>

      <div className="viewer-canvas-wrap">
        <button className="viewer-home-btn" onClick={recenterView} title="Recenter camera on robot">
          Home
        </button>
        <Canvas
          camera={{ position: [3.5, 2.5, 3.5], fov: 45 }}
          onCreated={({ camera }) => {
            cameraRef.current = camera;
          }}
        >
          <color attach="background" args={["#f4f8ff"]} />
          <ambientLight intensity={0.65} />
          <directionalLight position={[3, 5, 2]} intensity={1.1} />
          <pointLight position={[-3, 3, -3]} intensity={0.35} />

          <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]}>
            <planeGeometry args={[12, 12]} />
            <meshStandardMaterial color="#dbeafe" />
          </mesh>

          <Line points={[[0, 0, 0], [1, 0, 0]]} color="#ef4444" lineWidth={2} />
          <Line points={[[0, 0, 0], [0, 1, 0]]} color="#22c55e" lineWidth={2} />
          <Line points={[[0, 0, 0], [0, 0, 1]]} color="#3b82f6" lineWidth={2} />

          {urdfUrl ? <UrdfModel url={urdfUrl} packageRoot={urdfPackageRoot} currentQ={currentQ} onReadyState={setUrdfState} /> : null}
          {urdfUrl && urdfState === "loaded" ? null : <ProceduralRobot linkFrames={linkFrames} />}

          {!linkFrames.length ? (
            <Html center>
              <div className="viewer-empty">No FK chain available yet.</div>
            </Html>
          ) : null}

          <OrbitControls makeDefault ref={controlsRef} />
        </Canvas>
      </div>
    </section>
  );
}
