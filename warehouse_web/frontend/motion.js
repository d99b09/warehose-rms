// Interpolate only between received poses; never predict motion past telemetry.
export class RobotMotion {
  constructor() {
    this.poses = new Map();
  }

  update(robots, now, staleAfter) {
    const ids = new Set(robots.map(robot => robot.robot_id));
    for (const id of this.poses.keys()) {
      if (!ids.has(id)) this.poses.delete(id);
    }
    for (const robot of robots) {
      const previous = this.poses.get(robot.robot_id);
      if (![robot.x, robot.y, robot.theta].every(Number.isFinite)) {
        this.poses.delete(robot.robot_id);
        continue;
      }
      const fresh = !robot.stale && robot.odom_age !== null && robot.odom_age < staleAfter;
      if (fresh && previous?.fresh && now - previous.at < staleAfter * 1000 &&
          ['x', 'y', 'theta'].every(key => robot[key] === previous.to[key])) continue;
      const smooth = fresh && previous?.fresh && now - previous.at < staleAfter * 1000;
      this.poses.set(robot.robot_id, {
        from: smooth ? this.pose(robot, now) : robot,
        to: robot, at: now, fresh,
        duration: smooth ? Math.min(250, Math.max(16, now - previous.at)) : 0,
      });
    }
  }

  pose(robot, now) {
    const entry = this.poses.get(robot.robot_id);
    if (!entry) return robot;
    if (entry.duration === 0) return entry.to;
    const amount = Math.min(1, Math.max(0, (now - entry.at) / entry.duration));
    const angle = Math.atan2(Math.sin(entry.to.theta - entry.from.theta),
      Math.cos(entry.to.theta - entry.from.theta));
    return {
      x: entry.from.x + (entry.to.x - entry.from.x) * amount,
      y: entry.from.y + (entry.to.y - entry.from.y) * amount,
      theta: entry.from.theta + angle * amount,
    };
  }

  isAnimating(now) {
    return Array.from(this.poses.values()).some(entry => now < entry.at + entry.duration);
  }
}
