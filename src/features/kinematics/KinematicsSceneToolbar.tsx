import type {
  KinematicsCameraPreset,
  KinematicsFrameMode,
} from './teachingState'

export interface KinematicsSceneToolbarProps {
  cameraPreset: KinematicsCameraPreset
  frameMode: KinematicsFrameMode
  followStepCamera: boolean
  note: string
  onCameraPresetChange: (preset: KinematicsCameraPreset) => void
  onFrameModeChange: (mode: KinematicsFrameMode) => void
  onFollowStepCameraChange: (enabled: boolean) => void
  onResetCamera: () => void
}

const CAMERA_PRESETS: readonly [KinematicsCameraPreset, string][] = [
  ['overview', '空间总览'],
  ['top', '顶视图'],
  ['work-plane', '工作平面侧视图'],
  ['tool', '末端姿态视图'],
]

const FRAME_MODES: readonly [KinematicsFrameMode, string][] = [
  ['current', '当前坐标系'],
  ['chain', '坐标系链'],
  ['all', '全部坐标系'],
  ['hidden', '隐藏坐标系'],
]

export function KinematicsSceneToolbar({
  cameraPreset,
  frameMode,
  followStepCamera,
  note,
  onCameraPresetChange,
  onFrameModeChange,
  onFollowStepCameraChange,
  onResetCamera,
}: KinematicsSceneToolbarProps) {
  return (
    <aside aria-label="运动学三维教学控制" className="kinematics-scene-toolbar">
      <label>
        <span>视角</span>
        <select
          aria-label="教学相机预设"
          onChange={(event) => onCameraPresetChange(event.target.value as KinematicsCameraPreset)}
          value={cameraPreset}
        >
          {CAMERA_PRESETS.map(([preset, label]) => <option key={preset} value={preset}>{label}</option>)}
        </select>
      </label>
      <label>
        <span>坐标系</span>
        <select
          aria-label="坐标系显示模式"
          onChange={(event) => onFrameModeChange(event.target.value as KinematicsFrameMode)}
          value={frameMode}
        >
          {FRAME_MODES.map(([mode, label]) => <option key={mode} value={mode}>{label}</option>)}
        </select>
      </label>
      <label className="kinematics-scene-toolbar__follow">
        <input
          checked={followStepCamera}
          onChange={(event) => onFollowStepCameraChange(event.target.checked)}
          type="checkbox"
        />
        跟随推导视角
      </label>
      <button onClick={onResetCamera} type="button">回到本步视角</button>
      <p title={note}>{note}</p>
    </aside>
  )
}
