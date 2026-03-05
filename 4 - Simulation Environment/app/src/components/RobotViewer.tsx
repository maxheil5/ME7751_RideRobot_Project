import { Html, Line, OrbitControls } from "@react-three/drei";
import { Canvas } from "@react-three/fiber";
import { useEffect, useMemo, useState } from "react";
import type { Object3D } from "three";
import URDFLoader from "urdf-loader";

interface RobotViewerProps {
  linkFrames: number[][][];
  currentQ: number[];
  currentPoseText: string;
}

function UrdfModel({ url, onReadyState }: { url: string; onReadyState: (state: "loaded" | "failed") => void }) {
  const [robot, setRobot] = useState<Object3D | null>(null);

  useEffect(() => {
    const loader = new URDFLoader();
    loader.load(
      url,
      (loaded) => {
        setRobot(loaded);
        onReadyState("loaded");
      },
      undefined,
      () => {
        setRobot(null);
        onReadyState("failed");
      },
    );
  }, [url, onReadyState]);

  if (!robot) {
    return null;
  }

  return <primitive object={robot} scale={0.001} />;
}

function ProceduralRobot({ linkFrames }: { linkFrames: number[][][] }) {
  const points = useMemo(() => {
    const pts: [number, number, number][] = [[0, 0, 0]];
    for (const frame of linkFrames) {
      pts.push([frame[0][3] / 1000, frame[1][3] / 1000, frame[2][3] / 1000]);
    }
    return pts;
  }, [linkFrames]);

  const tcp = points[points.length - 1] || [0, 0, 0];

  return (
    <group>
      {points.map((point, idx) => (
        <mesh key={`joint-${idx}`} position={point}>
          <sphereGeometry args={[0.04, 20, 20]} />
          <meshStandardMaterial color={idx === points.length - 1 ? "#f97316" : "#0ea5e9"} roughness={0.4} metalness={0.2} />
        </mesh>
      ))}

      {points.slice(1).map((point, idx) => (
        <Line key={`link-${idx}`} points={[points[idx], point]} color="#38bdf8" lineWidth={3} />
      ))}

      <Line points={[tcp, [tcp[0] + 0.2, tcp[1], tcp[2]]]} color="#ef4444" lineWidth={2} />
      <Line points={[tcp, [tcp[0], tcp[1] + 0.2, tcp[2]]]} color="#22c55e" lineWidth={2} />
      <Line points={[tcp, [tcp[0], tcp[1], tcp[2] + 0.2]]} color="#3b82f6" lineWidth={2} />
    </group>
  );
}

export default function RobotViewer({ linkFrames, currentQ, currentPoseText }: RobotViewerProps) {
  const urdfUrl = import.meta.env.VITE_URDF_URL as string | undefined;
  const [urdfState, setUrdfState] = useState<"idle" | "loaded" | "failed">("idle");

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
        <Canvas camera={{ position: [3.5, 2.5, 3.5], fov: 45 }}>
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

          {urdfUrl ? <UrdfModel url={urdfUrl} onReadyState={setUrdfState} /> : null}
          <ProceduralRobot linkFrames={linkFrames} />

          {!linkFrames.length ? (
            <Html center>
              <div className="viewer-empty">No FK chain available yet.</div>
            </Html>
          ) : null}

          <OrbitControls makeDefault />
        </Canvas>
      </div>
    </section>
  );
}
