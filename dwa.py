"""Упрощённый DWA для автомобильной кинематики (единицы SI)."""
from __future__ import annotations
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class DWAResult:
    speed_mps: float
    steering_deg: float
    trajectory: tuple[tuple[float, float], ...]  # локальные координаты: right, forward
    score: float
    blocked: bool = False


def lidar_obstacles(ranges_mm, angle_offset_deg=180.0, max_range_mm=4000):
    """Возвращает локальные точки (right, forward), согласованные с LidarView."""
    points=[]
    n=len(ranges_mm)
    if not n: return points
    offset=math.radians(angle_offset_deg)
    for i, mm in enumerate(ranges_mm):
        if mm is None or mm <= 0 or mm > max_range_mm: continue
        a=2*math.pi*i/n+offset
        r=mm/1000.0
        points.append((math.sin(a)*r, math.cos(a)*r))
    return points


def choose_dwa_command(goal_local, obstacle_points, current_speed, max_speed=0.20,
                       max_accel=0.20, dt=0.2, horizon=2.0,
                       robot_radius=0.48, max_steering_deg=30.0, wheelbase=0.5):
    """Выбирает скорость/руление по цели, свободе траектории и плавности.

    goal_local=(right, forward). Кандидаты моделируются bicycle model.
    """
    gx, gy=goal_local
    if math.hypot(gx,gy) < 0.12:
        return DWAResult(0.0, 0.0, ((0.0,0.0),), 0.0, False)
    max_speed=max(0.0,float(max_speed))
    low=max(0.0,current_speed-max_accel*dt)
    high=min(max_speed,max(0.0,current_speed)+max_accel*dt)
    # При остановке разрешаем начать движение с нуля.
    speeds=sorted(set([0.0, low, (low+high)/2, high]))
    steerings=[-30,-20,-10,0,10,20,30]
    steerings=[s*max_steering_deg/30 for s in steerings]
    best=None
    for v in speeds:
        for steer in steerings:
            x=y=heading=0.0
            traj=[]
            clearance=10.0
            collision=False
            steps=max(1,int(horizon/dt))
            for _ in range(steps):
                heading += (v/wheelbase)*math.tan(math.radians(steer))*dt
                x += v*math.sin(heading)*dt
                y += v*math.cos(heading)*dt
                traj.append((x,y))
                for ox,oy in obstacle_points:
                    d=math.hypot(ox-x,oy-y)
                    clearance=min(clearance,d)
                    if d < robot_radius:
                        collision=True
                        break
                if collision: break
            if collision: continue
            ex=gx-x; ey=gy-y
            goal_dist=math.hypot(ex,ey)
            desired_heading=math.atan2(gx,gy)
            heading_error=abs((desired_heading-heading+math.pi)%(2*math.pi)-math.pi)
            # Reward progress and heading alignment, penalize steering changes and low clearance.
            score=2.0*goal_dist + 0.45*heading_error + 0.012*abs(steer) - 0.25*v
            score += 0.18/(max(0.05,clearance-robot_radius+0.05))
            if best is None or score < best.score:
                best=DWAResult(v,steer,tuple(traj),score,False)
    if best is None:
        return DWAResult(0.0,0.0,((0.0,0.0),),float('inf'),True)
    return best
